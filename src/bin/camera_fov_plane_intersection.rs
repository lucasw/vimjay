/// Take in a camera info and target frame and find the intersection of the boundary
/// of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
/// based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

use nalgebra::{Point3, Rotation, Rotation3};
use roslibrust::ros1::{NodeHandle, Publisher};
use std::collections::HashMap;
use tf_roslibrust::{
    TfError,
    TfListener,
    tf_util,
    transforms::isometry_from_transform,
};
use tokio::time::Duration;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// adapted from https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/undistort.dispatch.cpp
fn undistort_points(
    camera_info: &sensor_msgs::CameraInfo,
    points_in_camera: Vec<(f64, f64)>,
    max_count: u32,
) -> Vec<(f64, f64)> {

    let mut points2d = Vec::new();
    points2d.resize(points_in_camera.len(), (0.0, 0.0));

    // TODO(lucasw) tilt only used with large distortion matrices, support later
    // let inv_mat_tilt = Rotation3::identity();
    let mat_tilt: Rotation<f64, 3> = Rotation3::identity();

    let m = camera_info.K;

    let fx = m[0];
    let fy = m[4];
    let ifx = 1.0 / fx;
    let ify = 1.0 / fy;
    let cx = m[2];
    let cy = m[5];

    // distortion values are stored in a vector called 'k', not to be confused with camera_info.K
    // above
    const NUM: usize = 14;
    // TODO(lucasw) single line way to do this?
    let k = {
        let mut k: [f64; NUM] = [0.0; NUM];
        for i in 0..NUM {
            if i >= camera_info.D.len() {
                break;
            }
            k[i] = camera_info.D[i];
        }
        k
    };

    // TODO(lucasw) pass this in optionally
    let epsilon = 0.0;  // 1e-9;

    for (ind, (xo, yo)) in points_in_camera.iter().enumerate() {
        let u = xo;
        let v = yo;
        let mut x = (xo - cx) * ifx;
        let mut y = (yo - cy) * ify;

        // apply distortion
        if true {
            /*
            let vec_untilt = inv_mat_tilt * Point3::new(x, y, 1.0);
            let inv_proj = { if vec_untilt.z != 0.0 { 1. / vec_untilt.z } else { 1.0 } };
            x = inv_proj * vec_untilt.x;
            y = inv_proj * vec_untilt.y;
            */
            let x0 = x;
            let y0 = y;

            for j in 0..max_count {
                let r2 = x*x + y*y;
                let icdist = (1.0 + ((k[7]*r2 + k[6])*r2 + k[5])*r2) / (1.0 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
                if icdist < 0.0 { // test: undistortPoints.regression_14583
                    x = (u - cx)*ifx;
                    y = (v - cy)*ify;
                    print!("x");
                    break;
                }
                let delta_x = 2.0*k[2]*x*y + k[3] * (r2 + 2.0*x*x)+ k[8]*r2 +k[9]*r2*r2;
                let delta_y = k[2]*(r2 + 2.0*y*y) + 2.0*k[3]*x*y + k[10]*r2 + k[11]*r2*r2;
                x = (x0 - delta_x)*icdist;
                y = (y0 - delta_y)*icdist;

                if epsilon > 0.0 { // (criteria.type & cv::TermCriteria::EPS) {
                    let r2 = x*x + y*y;
                    let r4 = r2*r2;
                    let r6 = r4*r2;
                    let a1 = 2.0*x*y;
                    let a2 = r2 + 2.0*x*x;
                    let a3 = r2 + 2.0*y*y;
                    let cdist = 1.0 + k[0]*r2 + k[1]*r4 + k[4]*r6;
                    let icdist2 = 1.0 / (1.0 + k[5]*r2 + k[6]*r4 + k[7]*r6);

                    let xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2 + k[9]*r4;
                    let yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2 + k[11]*r4;

                    let vec_tilt = mat_tilt * Point3::new(xd0, yd0, 1.0);
                    let inv_proj = { if vec_tilt.z != 0.0 { 1.0 / vec_tilt.z } else { 1.0 } };
                    let xd = inv_proj * vec_tilt.x;
                    let yd = inv_proj * vec_tilt.y;

                    let x_proj = xd*fx + cx;
                    let y_proj = yd*fy + cy;

                    let error = ((x_proj - u).powi(2) + (y_proj - v).powi(2)).sqrt();
                    if error < epsilon { println!("{j} error {error:.12}"); break; }
                }
            }  // iterate until error is small
        }  // apply distortion

        /*
        let xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
        let yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
        let ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
        x = xx*ww;
        y = yy*ww;
        */

        points2d[ind].0 = x;
        points2d[ind].1 = y;
    }  // iterate through all points

    points2d
}

fn get_camera_edge_points(
    camera_info: &sensor_msgs::CameraInfo,
    num_per_edge: &u8,
) -> Vec<(f64, f64)> {
    let num_per_edge = *num_per_edge;

    let num_points = num_per_edge * 4 + 1;

    let mut points2d = Vec::new();
    points2d.resize(num_points.into(), (0.0, 0.0));
    let mut ind = 0;

    // create points in a loop around the edge of the image
    for i in 0..num_per_edge {
        let fr = i as f64 / num_per_edge as f64;
        points2d[ind].0 = fr * camera_info.width as f64;
        ind += 1;
    }

    for i in 0..num_per_edge {
        let fr = i as f64 / num_per_edge as f64;
        points2d[ind].0 = camera_info.width.into();
        points2d[ind].1 = fr * camera_info.height as f64;
        ind += 1;
    }

    for i in 0..num_per_edge {
        let fr = 1.0 - i as f64 / num_per_edge as f64;
        points2d[ind].0 = fr * camera_info.width as f64;
        points2d[ind].1 = camera_info.height.into();
        ind += 1;
    }

    // complete the loop with the + 1
    for i in 0..(num_per_edge + 1) {
        let fr = 1.0 - i as f64 / num_per_edge as f64;
        points2d[ind].1 = fr * camera_info.height as f64;
        ind += 1;
    }

    points2d
}

/*
fn camera_info_to_plane(
    // tf: &geometry_msgs::TransformStamped,
    tf: &tf_roslibrust::TransformStamped,
    camera_info: &sensor_msgs::CameraInfo,
    num_per_edge: &u8,
    target_frame: &str,
    output_frame: &str,
) {
    println!("{edge_points:?}");
}
*/

fn camera_info_edge_points_plane_intersection(
    listener: &TfListener,
    camera_info: &sensor_msgs::CameraInfo,
    num_per_edge: &u8,
    target_frame: &str,
    output_frame: &str,
    max_count: u32,
) -> Result<visualization_msgs::MarkerArray, TfError> {
    let t0 = tf_util::duration_now();

    let res0 = listener.lookup_transform(
        &target_frame,
        camera_info.header.frame_id.as_str(),
        Some(camera_info.header.stamp.clone()),
    );
    let camera_frame_to_target_plane_tfs = res0?;
    let camera_frame_to_target_plane = isometry_from_transform(&camera_frame_to_target_plane_tfs.transform);
    // println!("{camera_frame_to_target_plane}");

    let res1 = listener.lookup_transform(
        &output_frame,
        &target_frame,
        Some(camera_info.header.stamp.clone()),
    );
    let target_to_output_tfs = res1?;
    let target_to_output = isometry_from_transform(&camera_frame_to_target_plane_tfs.transform);

    let t1 = tf_util::duration_now();

    println!("have tf {:.3}s old",
        tf_util::duration_to_f64(t1 - tf_util::stamp_to_duration(&camera_info.header.stamp)));

    let edge_points_in_camera_2d = get_camera_edge_points(camera_info, num_per_edge);
    // TODO(lucasw) would be simpler to have undistort_points output into 3d
    let edge_points_ideal = undistort_points(&camera_info, edge_points_in_camera_2d, max_count);
    // println!("{edge_points_ideal:?}");

    let mut edge_points_in_camera_3d = Vec::new();
    // the extra +1 point is 0, 0, 0 and is the origin of the camera
    edge_points_in_camera_3d.resize(edge_points_ideal.len() + 1,
        Point3::new(0.0, 0.0, 0.0));
    for (ind, (xo, yo)) in edge_points_ideal.iter().enumerate() {
        edge_points_in_camera_3d[ind] = Point3::new(*xo, *yo, 1.0);
    }

    let mut edge_points_in_target = Vec::new();
    for pt3 in edge_points_in_camera_3d {  // .iter().enumerate() {
        let pt3_in_target = camera_frame_to_target_plane * pt3;
        edge_points_in_target.push(pt3_in_target);
    }

    // now find plane intersection
    let pt0 = edge_points_in_target.last().unwrap();
    let x0 = pt0.x;
    let y0 = pt0.y;
    let z0 = pt0.z;

    let mut points_in_plane = Vec::new();

    let mut is_full = true;
    for (ind, pt) in edge_points_in_target.iter().enumerate() {
        let x1 = pt.x;
        let y1 = pt.y;
        let z1 = pt.z;
        let non_intersecting = (z1 >= z0 && z0 >= 0.0) || (z1 <= z0 && z0 <= 0.0);
        if non_intersecting {
            is_full = false;
            // ROS_WARN_STREAM_THROTTLE(8.0, "non intersecting points");
            continue;
        }

        let intersect_distance = z0 / (z0 - z1);
        // the point intersecting the plane
        let x2 = x0 + (x1 - x0) * intersect_distance;
        let y2 = y0 + (y1 - y0) * intersect_distance;
        // this should be 0.0
        let z2 = z0 + (z1 - z0) * intersect_distance;

        points_in_plane.push(Point3::new(x2, y2, z2));
        // used_points2d_in_camera.push_back(points2d_in_camera[ind]);
    }

    let mut marker_array = visualization_msgs::MarkerArray::default();
    {
        let mut marker = visualization_msgs::Marker::default();
        marker.header = camera_info.header.clone();
        marker.header.frame_id = target_frame.to_string();
        marker.ns = "camera_fov".to_string();
        marker.r#type = 4;  // LINE_STRIP - TODO(lucasw) are those enums?
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.3;
        marker.color.g = 0.5;
        marker.color.b = 0.2;
        marker.color.a = 1.0;
        marker.scale.x = 0.07;
        marker.points.resize(points_in_plane.len(),
            geometry_msgs::Point::default());
        for (ind, pt3) in points_in_plane.iter().enumerate() {
            marker.points[ind].x = pt3.x;
            marker.points[ind].y = pt3.y;
            marker.points[ind].z = pt3.z;
        }
        marker_array.markers.push(marker);
    }

    // let t2 = tf_util::duration_now();
    print!("]");
    // println!("lookup: {:?}s, publish: {:?}s",
    //     tf_util::duration_to_f64(t1 - t0),
    //     tf_util::duration_to_f64(t2 - t1),
    // );

    Ok(marker_array)
}

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics

    // string parameters
    let mut param_str = HashMap::<String, String>::new();
    param_str.insert("target_frame".to_string(), "map".to_string());
    param_str.insert("output_frame".to_string(), "map".to_string());
    param_str.insert("max_count".to_string(), "8".to_string());
    // need extra leading _ for name and ns
    param_str.insert("_name".to_string(), "camera_fov_plane_intersection".to_string());
    param_str.insert("_ns".to_string(), "".to_string());

    // TODO(lucasw) can an existing rust arg handling library handle the ':=' ros cli args?
    let args = std::env::args();
    let mut args2 = Vec::new();
    for arg in args {
        let key_val: Vec<&str> = arg.split(":=").collect();
        if key_val.len() != 2 {
            args2.push(arg);
            continue;
        }

        let (mut key, val) = (key_val[0].to_string(), key_val[1].to_string());
        if !key.starts_with("_") {
            println!("unused arg pair {key}:={val}- need to prefix name with underscore");
            continue;
        }
        key.replace_range(0..1, "");

        if param_str.contains_key(&key) {
            param_str.insert(key, val);
        } else {
            println!("unused '{key}' '{val}'");
        }
    }
    println!("{args2:?}");
    println!("{param_str:?}");

    let ns = param_str.remove("_ns").unwrap();
    let target_frame = param_str.remove("target_frame").unwrap();
    let output_frame = param_str.remove("output_frame").unwrap();
    let max_count = param_str.remove("max_count").unwrap();
    let max_count = max_count.parse::<u32>().unwrap();
    println!("max count {max_count}");

    // TODO(lucasw) add support for ints via command line args
    let num_per_edge: u8 = 1;

    // TODO(lucasw) look for a '__name:=' argument
    let full_node_name = &format!(
        "/{}/{}",
        &ns,
        &param_str["_name"],
        ).replace("//", "/");
    println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await?;

    // TODO(lucasw) remember leading ns or the message won't transmit
    let marker_pub: Publisher<visualization_msgs::MarkerArray> = nh.advertise(&format!("{}/marker_array", ns.as_str()), 3).await?;

    let mut camera_info_sub = nh.subscribe::<sensor_msgs::CameraInfo>(&format!("{}/camera_info", ns.as_str()), 10).await?;

    let mut listener = TfListener::new(&nh).await;

    // TODO(lucasw) make this a queue
    let mut camera_info_q = None;
    let mut update_interval = tokio::time::interval(Duration::from_millis(50));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("ctrl-c exiting");
                break;
            }
            // TODO(lucasw) move this into listener
            rv = listener._dynamic_subscriber.next() => {
                // let t0 = tf_util::duration_now();
                print!(".");
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf(tfm);  // .await;
                    },
                    Some(Err(error)) => {
                        // probably can't keep up with tf publish rate
                        println!("{error}");
                    },
                    None => (),
                }
                // let t1 = tf_util::duration_now();
                // println!("{:?}", t1 - t0);
            }
            rv = listener._static_subscriber.next() => {
                // print!("+");
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf_static(tfm);  // .await;
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            }
            rv = camera_info_sub.next() => {
                print!("c");
                let t0 = tf_util::duration_now();
                let mut t1 = t0.clone();
                match rv {
                    Some(Ok(new_camera_info)) => {
                        if camera_info_q.is_none() {
                            camera_info_q = Some(new_camera_info);
                        } else {
                            println!("dropping new camera info");
                        }
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            }
            _ = update_interval.tick() => {
                if let Some(camera_info) = camera_info_q.clone() {
                    let update_rv = camera_info_edge_points_plane_intersection(
                        &listener, &camera_info,
                        &num_per_edge, &target_frame, &output_frame,
                        max_count,
                    );
                    match update_rv {
                        Ok(marker_array) => {
                            let pub_rv = marker_pub.publish(&marker_array).await;
                            match pub_rv {
                                Ok(()) => {},
                                Err(e) => { println!("{e}"); },
                            }
                            camera_info_q = None;
                        },
                        Err(err) => match err {
                            // expect the tf to arrive soon
                            TfError::AttemptedLookUpInFuture(_, _) => {
                                print!("-");
                            },
                            _ => {
                                println!("lookup error {err:?}");
                                camera_info_q = None;
                            },
                        },
                    }
                }
            },  // update
        }  // tokio select loop
    }

    Ok(())
}
