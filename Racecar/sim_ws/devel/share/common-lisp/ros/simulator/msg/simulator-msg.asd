
(cl:in-package :asdf)

(defsystem "simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CarControlData" :depends-on ("_package_CarControlData"))
    (:file "_package_CarControlData" :depends-on ("_package"))
    (:file "DriveValues" :depends-on ("_package_DriveValues"))
    (:file "_package_DriveValues" :depends-on ("_package"))
    (:file "PIDInput" :depends-on ("_package_PIDInput"))
    (:file "_package_PIDInput" :depends-on ("_package"))
  ))