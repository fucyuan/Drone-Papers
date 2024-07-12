;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::SimDebug)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'SimDebug (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::SIMDEBUG")
  (make-package "QUADROTOR_MSGS::SIMDEBUG"))

(in-package "ROS")
;;//! \htmlinclude SimDebug.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::SimDebug
  :super ros::object
  :slots (_header _pos_x _pos_y _pos_z _vel_x _vel_y _vel_z _pwm_1 _pwm_2 _pwm_3 _pwm_4 _omega_x _omega_y _omega_z ))

(defmethod quadrotor_msgs::SimDebug
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    ((:pos_z __pos_z) 0.0)
    ((:vel_x __vel_x) 0.0)
    ((:vel_y __vel_y) 0.0)
    ((:vel_z __vel_z) 0.0)
    ((:pwm_1 __pwm_1) 0.0)
    ((:pwm_2 __pwm_2) 0.0)
    ((:pwm_3 __pwm_3) 0.0)
    ((:pwm_4 __pwm_4) 0.0)
    ((:omega_x __omega_x) 0.0)
    ((:omega_y __omega_y) 0.0)
    ((:omega_z __omega_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   (setq _pos_z (float __pos_z))
   (setq _vel_x (float __vel_x))
   (setq _vel_y (float __vel_y))
   (setq _vel_z (float __vel_z))
   (setq _pwm_1 (float __pwm_1))
   (setq _pwm_2 (float __pwm_2))
   (setq _pwm_3 (float __pwm_3))
   (setq _pwm_4 (float __pwm_4))
   (setq _omega_x (float __omega_x))
   (setq _omega_y (float __omega_y))
   (setq _omega_z (float __omega_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:pos_z
   (&optional __pos_z)
   (if __pos_z (setq _pos_z __pos_z)) _pos_z)
  (:vel_x
   (&optional __vel_x)
   (if __vel_x (setq _vel_x __vel_x)) _vel_x)
  (:vel_y
   (&optional __vel_y)
   (if __vel_y (setq _vel_y __vel_y)) _vel_y)
  (:vel_z
   (&optional __vel_z)
   (if __vel_z (setq _vel_z __vel_z)) _vel_z)
  (:pwm_1
   (&optional __pwm_1)
   (if __pwm_1 (setq _pwm_1 __pwm_1)) _pwm_1)
  (:pwm_2
   (&optional __pwm_2)
   (if __pwm_2 (setq _pwm_2 __pwm_2)) _pwm_2)
  (:pwm_3
   (&optional __pwm_3)
   (if __pwm_3 (setq _pwm_3 __pwm_3)) _pwm_3)
  (:pwm_4
   (&optional __pwm_4)
   (if __pwm_4 (setq _pwm_4 __pwm_4)) _pwm_4)
  (:omega_x
   (&optional __omega_x)
   (if __omega_x (setq _omega_x __omega_x)) _omega_x)
  (:omega_y
   (&optional __omega_y)
   (if __omega_y (setq _omega_y __omega_y)) _omega_y)
  (:omega_z
   (&optional __omega_z)
   (if __omega_z (setq _omega_z __omega_z)) _omega_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _pos_x
    8
    ;; float64 _pos_y
    8
    ;; float64 _pos_z
    8
    ;; float64 _vel_x
    8
    ;; float64 _vel_y
    8
    ;; float64 _vel_z
    8
    ;; float64 _pwm_1
    8
    ;; float64 _pwm_2
    8
    ;; float64 _pwm_3
    8
    ;; float64 _pwm_4
    8
    ;; float64 _omega_x
    8
    ;; float64 _omega_y
    8
    ;; float64 _omega_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_z
       (sys::poke _pos_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_x
       (sys::poke _vel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_y
       (sys::poke _vel_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_z
       (sys::poke _vel_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pwm_1
       (sys::poke _pwm_1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pwm_2
       (sys::poke _pwm_2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pwm_3
       (sys::poke _pwm_3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pwm_4
       (sys::poke _pwm_4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _omega_x
       (sys::poke _omega_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _omega_y
       (sys::poke _omega_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _omega_z
       (sys::poke _omega_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _pos_x
     (setq _pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_y
     (setq _pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_z
     (setq _pos_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_x
     (setq _vel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_y
     (setq _vel_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_z
     (setq _vel_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pwm_1
     (setq _pwm_1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pwm_2
     (setq _pwm_2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pwm_3
     (setq _pwm_3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pwm_4
     (setq _pwm_4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _omega_x
     (setq _omega_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _omega_y
     (setq _omega_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _omega_z
     (setq _omega_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get quadrotor_msgs::SimDebug :md5sum-) "a8198ed55d087e47fb2498d93f1e3409")
(setf (get quadrotor_msgs::SimDebug :datatype-) "quadrotor_msgs/SimDebug")
(setf (get quadrotor_msgs::SimDebug :definition-)
      "Header header

float64 pos_x
float64 pos_y
float64 pos_z

float64 vel_x
float64 vel_y
float64 vel_z

float64 pwm_1
float64 pwm_2
float64 pwm_3
float64 pwm_4

float64 omega_x
float64 omega_y
float64 omega_z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :quadrotor_msgs/SimDebug "a8198ed55d087e47fb2498d93f1e3409")


