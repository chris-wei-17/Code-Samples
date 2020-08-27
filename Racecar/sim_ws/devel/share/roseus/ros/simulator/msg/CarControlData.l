;; Auto-generated. Do not edit!


(when (boundp 'simulator::CarControlData)
  (if (not (find-package "SIMULATOR"))
    (make-package "SIMULATOR"))
  (shadow 'CarControlData (find-package "SIMULATOR")))
(unless (find-package "SIMULATOR::CARCONTROLDATA")
  (make-package "SIMULATOR::CARCONTROLDATA"))

(in-package "ROS")
;;//! \htmlinclude CarControlData.msg.html


(defclass simulator::CarControlData
  :super ros::object
  :slots (_cv_speed _cv_turn_zero _cv_turn_speed _vp_speed _vp_turn_zero _vp_turn_speed _vesc_speed ))

(defmethod simulator::CarControlData
  (:init
   (&key
    ((:cv_speed __cv_speed) 0.0)
    ((:cv_turn_zero __cv_turn_zero) 0.0)
    ((:cv_turn_speed __cv_turn_speed) 0.0)
    ((:vp_speed __vp_speed) 0)
    ((:vp_turn_zero __vp_turn_zero) 0)
    ((:vp_turn_speed __vp_turn_speed) 0)
    ((:vesc_speed __vesc_speed) 0.0)
    )
   (send-super :init)
   (setq _cv_speed (float __cv_speed))
   (setq _cv_turn_zero (float __cv_turn_zero))
   (setq _cv_turn_speed (float __cv_turn_speed))
   (setq _vp_speed (round __vp_speed))
   (setq _vp_turn_zero (round __vp_turn_zero))
   (setq _vp_turn_speed (round __vp_turn_speed))
   (setq _vesc_speed (float __vesc_speed))
   self)
  (:cv_speed
   (&optional __cv_speed)
   (if __cv_speed (setq _cv_speed __cv_speed)) _cv_speed)
  (:cv_turn_zero
   (&optional __cv_turn_zero)
   (if __cv_turn_zero (setq _cv_turn_zero __cv_turn_zero)) _cv_turn_zero)
  (:cv_turn_speed
   (&optional __cv_turn_speed)
   (if __cv_turn_speed (setq _cv_turn_speed __cv_turn_speed)) _cv_turn_speed)
  (:vp_speed
   (&optional __vp_speed)
   (if __vp_speed (setq _vp_speed __vp_speed)) _vp_speed)
  (:vp_turn_zero
   (&optional __vp_turn_zero)
   (if __vp_turn_zero (setq _vp_turn_zero __vp_turn_zero)) _vp_turn_zero)
  (:vp_turn_speed
   (&optional __vp_turn_speed)
   (if __vp_turn_speed (setq _vp_turn_speed __vp_turn_speed)) _vp_turn_speed)
  (:vesc_speed
   (&optional __vesc_speed)
   (if __vesc_speed (setq _vesc_speed __vesc_speed)) _vesc_speed)
  (:serialization-length
   ()
   (+
    ;; float64 _cv_speed
    8
    ;; float64 _cv_turn_zero
    8
    ;; float64 _cv_turn_speed
    8
    ;; int16 _vp_speed
    2
    ;; int16 _vp_turn_zero
    2
    ;; int16 _vp_turn_speed
    2
    ;; float64 _vesc_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _cv_speed
       (sys::poke _cv_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cv_turn_zero
       (sys::poke _cv_turn_zero (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cv_turn_speed
       (sys::poke _cv_turn_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int16 _vp_speed
       (write-word _vp_speed s)
     ;; int16 _vp_turn_zero
       (write-word _vp_turn_zero s)
     ;; int16 _vp_turn_speed
       (write-word _vp_turn_speed s)
     ;; float64 _vesc_speed
       (sys::poke _vesc_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _cv_speed
     (setq _cv_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cv_turn_zero
     (setq _cv_turn_zero (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cv_turn_speed
     (setq _cv_turn_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int16 _vp_speed
     (setq _vp_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _vp_turn_zero
     (setq _vp_turn_zero (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _vp_turn_speed
     (setq _vp_turn_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _vesc_speed
     (setq _vesc_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get simulator::CarControlData :md5sum-) "324d1d835c12674bf092534c6d2533e0")
(setf (get simulator::CarControlData :datatype-) "simulator/CarControlData")
(setf (get simulator::CarControlData :definition-)
      "# Constants from controlling the car via node

# cmd_vel
float64 cv_speed

float64 cv_turn_zero
float64 cv_turn_speed

# drive_pwm
int16 vp_speed

int16 vp_turn_zero
int16 vp_turn_speed

# VESC
float64 vesc_speed

")



(provide :simulator/CarControlData "324d1d835c12674bf092534c6d2533e0")


