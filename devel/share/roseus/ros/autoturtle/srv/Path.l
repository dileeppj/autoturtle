;; Auto-generated. Do not edit!


(when (boundp 'autoturtle::Path)
  (if (not (find-package "AUTOTURTLE"))
    (make-package "AUTOTURTLE"))
  (shadow 'Path (find-package "AUTOTURTLE")))
(unless (find-package "AUTOTURTLE::PATH")
  (make-package "AUTOTURTLE::PATH"))
(unless (find-package "AUTOTURTLE::PATHREQUEST")
  (make-package "AUTOTURTLE::PATHREQUEST"))
(unless (find-package "AUTOTURTLE::PATHRESPONSE")
  (make-package "AUTOTURTLE::PATHRESPONSE"))

(in-package "ROS")





(defclass autoturtle::PathRequest
  :super ros::object
  :slots (_start _stop ))

(defmethod autoturtle::PathRequest
  (:init
   (&key
    ((:start __start) (make-array 0 :initial-element 0 :element-type :integer))
    ((:stop __stop) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _start __start)
   (setq _stop __stop)
   self)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:stop
   (&optional __stop)
   (if __stop (setq _stop __stop)) _stop)
  (:serialization-length
   ()
   (+
    ;; int32[] _start
    (* 4    (length _start)) 4
    ;; int32[] _stop
    (* 4    (length _stop)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _start
     (write-long (length _start) s)
     (dotimes (i (length _start))
       (write-long (elt _start i) s)
       )
     ;; int32[] _stop
     (write-long (length _stop) s)
     (dotimes (i (length _stop))
       (write-long (elt _stop i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _start i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _stop
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stop (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _stop i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass autoturtle::PathResponse
  :super ros::object
  :slots (_path ))

(defmethod autoturtle::PathResponse
  (:init
   (&key
    ((:path __path) "")
    )
   (send-super :init)
   (setq _path (string __path))
   self)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:serialization-length
   ()
   (+
    ;; string _path
    4 (length _path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _path
       (write-long (length _path) s) (princ _path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass autoturtle::Path
  :super ros::object
  :slots ())

(setf (get autoturtle::Path :md5sum-) "d32b3108e0653dfb8af9c9692bc47b31")
(setf (get autoturtle::Path :datatype-) "autoturtle/Path")
(setf (get autoturtle::Path :request) autoturtle::PathRequest)
(setf (get autoturtle::Path :response) autoturtle::PathResponse)

(defmethod autoturtle::PathRequest
  (:response () (instance autoturtle::PathResponse :init)))

(setf (get autoturtle::PathRequest :md5sum-) "d32b3108e0653dfb8af9c9692bc47b31")
(setf (get autoturtle::PathRequest :datatype-) "autoturtle/PathRequest")
(setf (get autoturtle::PathRequest :definition-)
      "int32[] start
int32[] stop
---
string path
# int32[] path
")

(setf (get autoturtle::PathResponse :md5sum-) "d32b3108e0653dfb8af9c9692bc47b31")
(setf (get autoturtle::PathResponse :datatype-) "autoturtle/PathResponse")
(setf (get autoturtle::PathResponse :definition-)
      "int32[] start
int32[] stop
---
string path
# int32[] path
")



(provide :autoturtle/Path "d32b3108e0653dfb8af9c9692bc47b31")


