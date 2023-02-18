cmake_minimum_required(VERSION 2.8.3)
project(rbn100_msgs)
find_package(catkin REQUIRED 
                    COMPONENTS std_msgs 
                               actionlib_msgs 
                               message_generation)

add_message_files(DIRECTORY msg
                      FILES BumperEvent.msg
                            CliffEvent.msg
                            KeyboardInput.msg
                            MotorPower.msg
                            Encoder.msg
                            Ultra.msg
                            StereoImage.msg
                            WheelSpeed.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
