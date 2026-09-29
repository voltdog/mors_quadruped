lcm-gen -p -x --cpp-std c++11 -j *.lcm

javac -cp $LCM_JAR mors_msgs/*.java
jar cf lcm_types.jar mors_msgs/*.class