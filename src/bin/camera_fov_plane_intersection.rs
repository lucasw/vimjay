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

roslibrust_codegen_macro::find_and_generate_ros_messages!();

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics

    // string parameters
    let mut param_str = HashMap::<String, String>::new();
    param_str.insert("target_frame".to_string(), "map".to_string());
    param_str.insert("output_frame".to_string(), "map".to_string());
    // need extra leading _ for name and ns
    param_str.insert("_name".to_string(), "camera_info_to_plane".to_string());
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

    // TODO(lucasw) add support for ints via command line args
    let num_per_edge = 1;

    // TODO(lucasw) look for a '__name:=' argument
    let full_node_name = &format!(
        "/{}/{}",
        &ns,
        &param_str["_name"],
        ).replace("//", "/");
    println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await?;

    let tf_pub: Publisher<visualization_msgs::MarkerArray> = nh.advertise("marker_array", 3).await?;

    let mut camera_info_sub = nh.subscribe::<sensor_msgs::CameraInfo>(&format!("{}/camera_info", ns.as_str()), 10).await?;

    let mut listener = TfListener::new(&nh).await;

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
                print!("[");
                let t0 = tf_util::duration_now();
                let mut t1 = t0.clone();
                match rv {
                    Some(Ok(camera_info)) => {
                        // println!("{camera_info:?}");
                        // need to loop and tokio sleep until this transform is available
                        for i in 0..10 {
                            let res = listener.lookup_transform(
                                &param_str["target_frame"],
                                camera_info.header.frame_id.as_str(),
                                Some(camera_info.header.stamp.clone()),
                            );
                            t1 = tf_util::duration_now();
                            match res {
                                Ok(tf) => {
                                    println!("have tf {i} {tf:?}");
                                    break;
                                },
                                Err(err) => match err {
                                    TfError::AttemptedLookUpInFuture(_, _) => {
                                        print!("-");
                                        // TODO(lucaw) this doesn't allow more tf messages to
                                        // arrive
                                        tokio::time::sleep(Duration::from_millis(50)).await;
                                    },
                                    _ => {
                                        println!("lookup error {t1:?} {t0:?} {err:?}");
                                        break;
                                    },
                                },
                            }
                        }  // loop until lookup works or times out or fails
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    // TODO(lucasw) should this ever happen?
                    None => (),
                }
                let t2 = tf_util::duration_now();
                print!("]");
                println!("lookup: {:?}s, publish: {:?}s",
                    tf_util::duration_to_f64(t1 - t0),
                    tf_util::duration_to_f64(t2 - t1),
                );
            }
        }  // tokio select loop
    }

    Ok(())
}
