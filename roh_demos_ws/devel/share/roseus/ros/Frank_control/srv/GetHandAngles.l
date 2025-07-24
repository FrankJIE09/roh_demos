;; Auto-generated. Do not edit!


(when (boundp 'Frank_control::GetHandAngles)
  (if (not (find-package "FRANK_CONTROL"))
    (make-package "FRANK_CONTROL"))
  (shadow 'GetHandAngles (find-package "FRANK_CONTROL")))
(unless (find-package "FRANK_CONTROL::GETHANDANGLES")
  (make-package "FRANK_CONTROL::GETHANDANGLES"))
(unless (find-package "FRANK_CONTROL::GETHANDANGLESREQUEST")
  (make-package "FRANK_CONTROL::GETHANDANGLESREQUEST"))
(unless (find-package "FRANK_CONTROL::GETHANDANGLESRESPONSE")
  (make-package "FRANK_CONTROL::GETHANDANGLESRESPONSE"))

(in-package "ROS")





(defclass Frank_control::GetHandAnglesRequest
  :super ros::object
  :slots ())

(defmethod Frank_control::GetHandAnglesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass Frank_control::GetHandAnglesResponse
  :super ros::object
  :slots (_thumb_bend _index_bend _middle_bend _ring_bend _pinky_bend _thumb_rot _detected ))

(defmethod Frank_control::GetHandAnglesResponse
  (:init
   (&key
    ((:thumb_bend __thumb_bend) 0.0)
    ((:index_bend __index_bend) 0.0)
    ((:middle_bend __middle_bend) 0.0)
    ((:ring_bend __ring_bend) 0.0)
    ((:pinky_bend __pinky_bend) 0.0)
    ((:thumb_rot __thumb_rot) 0.0)
    ((:detected __detected) nil)
    )
   (send-super :init)
   (setq _thumb_bend (float __thumb_bend))
   (setq _index_bend (float __index_bend))
   (setq _middle_bend (float __middle_bend))
   (setq _ring_bend (float __ring_bend))
   (setq _pinky_bend (float __pinky_bend))
   (setq _thumb_rot (float __thumb_rot))
   (setq _detected __detected)
   self)
  (:thumb_bend
   (&optional __thumb_bend)
   (if __thumb_bend (setq _thumb_bend __thumb_bend)) _thumb_bend)
  (:index_bend
   (&optional __index_bend)
   (if __index_bend (setq _index_bend __index_bend)) _index_bend)
  (:middle_bend
   (&optional __middle_bend)
   (if __middle_bend (setq _middle_bend __middle_bend)) _middle_bend)
  (:ring_bend
   (&optional __ring_bend)
   (if __ring_bend (setq _ring_bend __ring_bend)) _ring_bend)
  (:pinky_bend
   (&optional __pinky_bend)
   (if __pinky_bend (setq _pinky_bend __pinky_bend)) _pinky_bend)
  (:thumb_rot
   (&optional __thumb_rot)
   (if __thumb_rot (setq _thumb_rot __thumb_rot)) _thumb_rot)
  (:detected
   (&optional (__detected :null))
   (if (not (eq __detected :null)) (setq _detected __detected)) _detected)
  (:serialization-length
   ()
   (+
    ;; float32 _thumb_bend
    4
    ;; float32 _index_bend
    4
    ;; float32 _middle_bend
    4
    ;; float32 _ring_bend
    4
    ;; float32 _pinky_bend
    4
    ;; float32 _thumb_rot
    4
    ;; bool _detected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _thumb_bend
       (sys::poke _thumb_bend (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _index_bend
       (sys::poke _index_bend (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _middle_bend
       (sys::poke _middle_bend (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ring_bend
       (sys::poke _ring_bend (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pinky_bend
       (sys::poke _pinky_bend (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thumb_rot
       (sys::poke _thumb_rot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _detected
       (if _detected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _thumb_bend
     (setq _thumb_bend (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _index_bend
     (setq _index_bend (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _middle_bend
     (setq _middle_bend (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ring_bend
     (setq _ring_bend (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pinky_bend
     (setq _pinky_bend (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thumb_rot
     (setq _thumb_rot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _detected
     (setq _detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass Frank_control::GetHandAngles
  :super ros::object
  :slots ())

(setf (get Frank_control::GetHandAngles :md5sum-) "ae6e268c4bfe9632c21995915fa1b5ce")
(setf (get Frank_control::GetHandAngles :datatype-) "Frank_control/GetHandAngles")
(setf (get Frank_control::GetHandAngles :request) Frank_control::GetHandAnglesRequest)
(setf (get Frank_control::GetHandAngles :response) Frank_control::GetHandAnglesResponse)

(defmethod Frank_control::GetHandAnglesRequest
  (:response () (instance Frank_control::GetHandAnglesResponse :init)))

(setf (get Frank_control::GetHandAnglesRequest :md5sum-) "ae6e268c4bfe9632c21995915fa1b5ce")
(setf (get Frank_control::GetHandAnglesRequest :datatype-) "Frank_control/GetHandAnglesRequest")
(setf (get Frank_control::GetHandAnglesRequest :definition-)
      "---
float32 thumb_bend
float32 index_bend
float32 middle_bend
float32 ring_bend
float32 pinky_bend
float32 thumb_rot
bool detected 
")

(setf (get Frank_control::GetHandAnglesResponse :md5sum-) "ae6e268c4bfe9632c21995915fa1b5ce")
(setf (get Frank_control::GetHandAnglesResponse :datatype-) "Frank_control/GetHandAnglesResponse")
(setf (get Frank_control::GetHandAnglesResponse :definition-)
      "---
float32 thumb_bend
float32 index_bend
float32 middle_bend
float32 ring_bend
float32 pinky_bend
float32 thumb_rot
bool detected 
")



(provide :Frank_control/GetHandAngles "ae6e268c4bfe9632c21995915fa1b5ce")


