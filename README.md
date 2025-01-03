# depth_pro_ros
A depth Pro wrapper for ROS2
The depth pro model out  of the box predicts focal length and the metric depth (z dimension) for any image with an accuracy of about 90%.
TO DO
Still Under development
- Working on reducing the inference time for the depth_pro model to be capable for real time inference on resource constrained environment
- Tested  on Alienware Laptop 4090RTX Nvidia GPU
 Models 
- Suggestion to Improve Inference Time:
-  Perform Model Prunning
-  Model Quantization and Distillation
-  Make use of Flash Attention Technique to improve model
