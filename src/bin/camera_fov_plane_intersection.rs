/// Take in a camera info and target frame and find the intersection of the boundary
/// of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
/// based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

use roslibrust::ros1::{NodeHandle, Publisher};
use std::collections::HashMap;
use tf_roslibrust::{
    TfError,
    TfListener,
    tf_util,
};
use tokio::time::Duration;
use vimjay::camera_info_edge_points_plane_intersection;
use tf_roslibrust::transforms::{sensor_msgs, visualization_msgs};

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
    let latching = false;
    let marker_pub: Publisher<visualization_msgs::MarkerArray> = nh.advertise(&format!("{}/marker_array", ns.as_str()), 3, latching).await?;

    let mut camera_info_sub = nh.subscribe::<sensor_msgs::CameraInfo>(&format!("{}/camera_info", ns.as_str()), 30).await?;

    let listener = TfListener::new(&nh).await;

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        // TODO(lucasw) give the msg receiver thread a chance to cleanly finish the mcap
        println!("ctrl-c, exiting");
        std::process::exit(0);
    });

    loop {
        // TODO(lucasw this will never return if camera_infos stop arriving
        if listener.is_finished() {
            break;
        }

        // TODO(lucasw) try_next + a sleep instead to allow the check for is_finished
        let rv = camera_info_sub.next().await;
        print!("c");
        match rv {
            Some(Ok(camera_info)) => {
                let t0 = tf_util::duration_now();
                // keep trying until the lookup works or timeout expires
                // TODO(lucasw) check if the camera_info stamp is too old, or too far in the
                // future relative to what's in the tf_buffer (may need new tf buffer methods
                // and skip this camera_info if it is out of bounds
                loop {
                    if listener.is_finished() {
                        break;
                    }

                    let update_rv = camera_info_edge_points_plane_intersection(
                        &listener, &camera_info,
                        &num_per_edge, &target_frame, &output_frame,
                        max_count,
                    );
                    match update_rv {
                        Ok(marker_array) => {
                            let tdiff = tf_util::duration_now() - t0;
                            print!(" {:.3}s elapsed ", tf_util::duration_to_f64(tdiff));
                            marker_pub.publish(&marker_array).await?;
                            break;
                        },
                        Err(err) => match err {
                            // expect the tf to arrive soon
                            // the 'future' here means that the lookup is later than available
                            // transforms in the tf buffer
                            TfError::AttemptedLookUpInFuture(_, _) => {
                                print!("-");
                                // keep looping until time runs out-
                                // but in that case the lookup is either not in the
                                // future any longer, or it was too far in the future
                                // and should have given up immediately
                                tokio::time::sleep(Duration::from_millis(20)).await;
                            },
                            _ => {
                                println!("lookup error {err:?}");
                                break;
                            },
                        },
                    }
                }
            },
            Some(Err(error)) => {
                // TODO(lucasw) return the error normally
                panic!("{error}");
            },
            None => (),
        }
    }

    Ok(())
}
