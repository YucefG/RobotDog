;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::CheaterState)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'CheaterState (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::CHEATERSTATE")
  (make-package "UNITREE_LEGGED_MSGS::CHEATERSTATE"))

(in-package "ROS")
;;//! \htmlinclude CheaterState.msg.html


(defclass unitree_legged_msgs::CheaterState
  :super ros::object
  :slots (_orientation _position _vBody _vWorld _omegaBody _acceleration ))

(defmethod unitree_legged_msgs::CheaterState
  (:init
   (&key
    ((:orientation __orientation) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vBody __vBody) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vWorld __vWorld) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:omegaBody __omegaBody) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:acceleration __acceleration) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _orientation __orientation)
   (setq _position __position)
   (setq _vBody __vBody)
   (setq _vWorld __vWorld)
   (setq _omegaBody __omegaBody)
   (setq _acceleration __acceleration)
   self)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:vBody
   (&optional __vBody)
   (if __vBody (setq _vBody __vBody)) _vBody)
  (:vWorld
   (&optional __vWorld)
   (if __vWorld (setq _vWorld __vWorld)) _vWorld)
  (:omegaBody
   (&optional __omegaBody)
   (if __omegaBody (setq _omegaBody __omegaBody)) _omegaBody)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; float32[4] _orientation
    (* 4    4)
    ;; float32[3] _position
    (* 4    3)
    ;; float32[3] _vBody
    (* 4    3)
    ;; float32[3] _vWorld
    (* 4    3)
    ;; float32[3] _omegaBody
    (* 4    3)
    ;; float32[3] _acceleration
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _orientation
     (dotimes (i 4)
       (sys::poke (elt _orientation i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _vBody
     (dotimes (i 3)
       (sys::poke (elt _vBody i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _vWorld
     (dotimes (i 3)
       (sys::poke (elt _vWorld i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _omegaBody
     (dotimes (i 3)
       (sys::poke (elt _omegaBody i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _acceleration
     (dotimes (i 3)
       (sys::poke (elt _acceleration i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _orientation
   (dotimes (i (length _orientation))
     (setf (elt _orientation i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _vBody
   (dotimes (i (length _vBody))
     (setf (elt _vBody i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _vWorld
   (dotimes (i (length _vWorld))
     (setf (elt _vWorld i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _omegaBody
   (dotimes (i (length _omegaBody))
     (setf (elt _omegaBody i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _acceleration
   (dotimes (i (length _acceleration))
     (setf (elt _acceleration i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get unitree_legged_msgs::CheaterState :md5sum-) "f8cc1254d33feda52dbc17fc7e4c7459")
(setf (get unitree_legged_msgs::CheaterState :datatype-) "unitree_legged_msgs/CheaterState")
(setf (get unitree_legged_msgs::CheaterState :definition-)
      "float32[4] orientation
float32[3] position
float32[3] vBody
float32[3] vWorld
float32[3] omegaBody
float32[3] acceleration

")



(provide :unitree_legged_msgs/CheaterState "f8cc1254d33feda52dbc17fc7e4c7459")


