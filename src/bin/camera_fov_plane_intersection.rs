/// Take in a camera info and target frame and find the intersection of the boundary
/// of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
/// based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

// use nalgebra::base::Vector3;
use roslibrust::ros1::{NodeHandle, Publisher};
use std::collections::HashMap;
use tf_roslibrust::{
    TfError,
    TfListener,
    tf_util,
};
use tokio::time::Duration;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// adapted from https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/undistort.dispatch.cpp
fn undistort_points(
    camera_info: &sensor_msgs::CameraInfo,
    points_in_camera: Vec<(f64, f64)>,
) -> Vec<(f64, f64)> {

    let mut points2d = Vec::new();
    points2d.resize(points_in_camera.len(), (0.0, 0.0));

    let fx = camera_info.K[0];
    let fy = camera_info.K[4];
    let ifx = 1.0 / fx;
    let ify = 1.0 / fy;
    let cx = camera_info.K[2];
    let cy = camera_info.K[5];

    for (ind, (xo, yo)) in points_in_camera.iter().enumerate() {
        // let u = xo;
        // let v = yo;
        let x = (xo - cx) * ifx;
        let y = (yo - cy) * ify;

        // TODO(lucasw) apply distortion

        points2d[ind].0 = x;
        points2d[ind].1 = y;
    }

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

fn update(
    listener: &TfListener,
    camera_info: &sensor_msgs::CameraInfo,
    num_per_edge: &u8,
    target_frame: &str,
    output_frame: &str,
) -> Result<visualization_msgs::MarkerArray, TfError> {
    let t0 = tf_util::duration_now();

    let res0 = listener.lookup_transform(
        &target_frame,
        camera_info.header.frame_id.as_str(),
        Some(camera_info.header.stamp.clone()),
    );
    let camera_frame_to_target_plane_tfs = res0?;

    let res1 = listener.lookup_transform(
        &output_frame,
        &target_frame,
        Some(camera_info.header.stamp.clone()),
    );
    let target_to_output_tfs = res1?;

    let t1 = tf_util::duration_now();

    println!("have tf {:.3}s old",
        tf_util::duration_to_f64(t1 - tf_util::stamp_to_duration(&camera_info.header.stamp)));

    let edge_points_in_camera_2d = get_camera_edge_points(camera_info, num_per_edge);
    // TODO(lucasw) would be simpler to have undistort_points output into 3d
    let edge_points_ideal = undistort_points(&camera_info, edge_points_in_camera_2d);
    // println!("{edge_points_ideal:?}");

    let mut edge_points_in_camera_3d = Vec::new();
    // the extra +1 point is 0, 0, 0 and is the origin of the camera
    edge_points_in_camera_3d.resize(edge_points_ideal.len() + 1, (0.0, 0.0, 0.0));
    for (ind, (xo, yo)) in edge_points_ideal.iter().enumerate() {
        edge_points_in_camera_3d[ind] = (*xo, *yo, 1.0);
    }

    let mut marker_array = visualization_msgs::MarkerArray::default();
    {
        let mut marker = visualization_msgs::Marker::default();
        marker.header = camera_info.header.clone();
        marker.ns = "camera_fov".to_string();
        marker.r#type = 4;  // LINE_STRIP - TODO(lucasw) are those enums?
        marker.points.resize(edge_points_in_camera_3d.len(),
            geometry_msgs::Point::default());  // { x: 0.0, y: 0.0, z: 0.0, };
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.3;
        marker.color.g = 0.5;
        marker.color.b = 0.2;
        marker.color.a = 1.0;
        marker.scale.x = 0.07;
        for (ind, (xo, yo, zo)) in edge_points_in_camera_3d.iter().enumerate() {
            marker.points[ind].x = *xo;
            marker.points[ind].y = *yo;
            marker.points[ind].z = *zo;
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
                    let update_rv = update(&listener, &camera_info,
                        &num_per_edge, &target_frame, &output_frame);
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
