
(cl:in-package :asdf)

(defsystem "Frank_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetHandAngles" :depends-on ("_package_GetHandAngles"))
    (:file "_package_GetHandAngles" :depends-on ("_package"))
  ))