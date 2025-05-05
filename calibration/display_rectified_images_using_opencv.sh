DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/zed_zed_node_left_rect_color.jpg --intrinsics_calibration $DIR/zed_zed_node_left_rect_color.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_mid.jpg --intrinsics_calibration $DIR/camera_rear_mid.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_side_right.jpg --intrinsics_calibration $DIR/camera_side_right.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_side_left.jpg --intrinsics_calibration $DIR/camera_side_left.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_right.jpg --intrinsics_calibration $DIR/camera_rear_right.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_left.jpg --intrinsics_calibration $DIR/camera_rear_left.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/zed_zed_node_right_rect_color.jpg --intrinsics_calibration $DIR/zed_zed_node_right_rect_color.intrinsics.yaml
