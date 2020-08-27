;; Auto-generated. Do not edit!


(when (boundp 'simulator::DriveValues)
  (if (not (find-package "SIMULATOR"))
    (make-package "SIMULATOR"))
  (shadow 'DriveValues (find-package "SIMULATOR")))
(unless (find-package "SIMULATOR::DRIVEVALUES")
  (make-package "SIMULATOR::DRIVEVALUES"))

(in-package "ROS")
;;//! \htmlinclude DriveValues.msg.html


(defclass simulator::DriveValues
  :super ros::object
  :slots (_pwm_drive _pwm_angle ))

(defmethod simulator::DriveValues
  (:init
   (&key
    ((:pwm_drive __pwm_drive) 0)
    ((:pwm_angle __pwm_angle) 0)
    )
   (send-super :init)
   (setq _pwm_drive (round __pwm_drive))
   (setq _pwm_angle (round __pwm_angle))
   self)
  (:pwm_drive
   (&optional __pwm_drive)
   (if __pwm_drive (setq _pwm_drive __pwm_drive)) _pwm_drive)
  (:pwm_angle
   (&optional __pwm_angle)
   (if __pwm_angle (setq _pwm_angle __pwm_angle)) _pwm_angle)
  (:serialization-length
   ()
   (+
    ;; int16 _pwm_drive
    2
    ;; int16 _pwm_angle
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _pwm_drive
       (write-word _pwm_drive s)
     ;; int16 _pwm_angle
       (write-word _pwm_angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _pwm_drive
     (setq _pwm_drive (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _pwm_angle
     (setq _pwm_angle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get simulator::DriveValues :md5sum-) "180768e2d6cce7b3f71749adb84f7b29")
(setf (get simulator::DriveValues :datatype-) "simulator/DriveValues")
(setf (get simulator::DriveValues :definition-)
      "# PWM duty cycle (0-100%) corresponds to (0-65535) interval
int16 pwm_drive
int16 pwm_angle

")



(provide :simulator/DriveValues "180768e2d6cce7b3f71749adb84f7b29")


