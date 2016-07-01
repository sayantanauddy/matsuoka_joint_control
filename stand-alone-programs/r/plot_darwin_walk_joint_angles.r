library(ggplot2)
library(reshape2)
library(corrplot)

mydata <- read.csv("/home/sayantan/ros_workspaces/catkin_ws/src/robot_walking/data/darwin_walk_demo_joint_angles.csv", 
                     header=TRUE, sep=",")

time <- mydata$time
j_pelvis_l <- mydata$j_pelvis_l
j_thigh1_l <- mydata$j_thigh1_l
j_thigh2_l <- mydata$j_thigh2_l
j_tibia_l  <- mydata$j_tibia_l
j_ankle1_l <- mydata$j_ankle1_l
j_ankle2_l <- mydata$j_ankle2_l
j_thigh1_r <- mydata$j_thigh1_r
j_thigh2_r <- mydata$j_thigh2_r
j_tibia_r  <- mydata$j_tibia_r
j_ankle1_r <- mydata$j_ankle1_r
j_ankle2_r <- mydata$j_ankle2_r

mydata_long <- melt(mydata, id="time")  # convert to long format

ggplot(data=mydata_long, aes(x=time, y=value, colour=variable, fill = "white")) + 
  geom_line() +
  theme(panel.background = element_rect(fill = 'white', colour = 'black'))

M <- cor(mydata)
corrplot(M, method="number")


