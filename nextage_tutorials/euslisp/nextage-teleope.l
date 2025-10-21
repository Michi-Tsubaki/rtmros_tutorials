#!/usr/bin/env roseus

;; Load and Initialize roseus program
(ros::load-ros-manifest "roseus")
(ros::roseus "surgical-nextage-manekin")

;; Import msgs.
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")

;; Import Robot Interface
(require "package://jsk_surgical_nextage_robot/euslisp/surgical-nextage-interface.l")

(defun apply-deadband (value threshold)
  (if (< (abs value) threshold)
      0.0
    value))

(defun apply-deadband-to-axes (axes threshold)
  (let ((result (copy-seq axes)))
    (dotimes (i (length axes))
      (setf (elt result i) (apply-deadband (elt axes i) threshold)))
    result))

;; Callback for left spacenav
(defun left-cb (msg)
  (setq *ljoy-raw* (send msg :axes))
  (setq *ljoy* (apply-deadband-to-axes *ljoy-raw* 0.00))
  (setq *lbot* (send msg :buttons))
  )

;; Callback for right spacenav
(defun right-cb (msg)
  (setq *rjoy-raw* (send msg :axes))
  (setq *rjoy* (apply-deadband-to-axes *rjoy-raw* 0.05))
  (setq *rbot* (send msg :buttons))
  )

;; Initialization
(nextage-init)
(objects (list *nextage*))

;; Global params for left arm
(setq *l-forceps-rotation* 2.4) ;; This param is to be fixed.
(setq *l-grasp-position* t)

;; Global params for right arm
(setq *r-forceps-rotation* -2.4) ;; This param is to be fixed.
(setq *r-grasp-position* t)

;; Reset pose
(send *nextage* :reset-pose)
(send *nextage* :head :angle-vector #f(0 30))
(send *ri* :angle-vector (send *nextage* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; Initialize joystick values
(setq *ljoy-raw* #f(0 0 0 0 0 0))
(setq *ljoy* #f(0 0 0 0 0 0))
(setq *lbot* #f(0 0))
(setq *rjoy-raw* #f(0 0 0 0 0 0))
(setq *rjoy* #f(0 0 0 0 0 0))
(setq *rbot* #f(0 0))

;; Subscribe to spacenav topics
(ros::subscribe "spacenav/joy" sensor_msgs::Joy #'left-cb)
(ros::subscribe "/right_manip/spacenav/joy" sensor_msgs::Joy #'right-cb)

;; Common scales for both arms
(setq *scale* 15)
(setq *scale-pitch* 0.02)
(setq *scale-yaw* 1)
(setq *deadband-threshold* 0.04)

(do-until-key
 (ros::spin-once)
 
 (format t "Left Joy Raw: ~A~%" *ljoy-raw*)
 (format t "Left Joy Filtered: ~A~%" *ljoy*)
 (format t "Right Joy Raw: ~A~%" *rjoy-raw*)
 (format t "Right Joy Filtered: ~A~%" *rjoy*)
 
 ;; Left arm control
 (setq *ljoyx* (* (elt *ljoy* 0) *scale*))
 (setq *ljoyy* (* (elt *ljoy* 1) *scale*))
 (setq *ljoyz* (* (elt *ljoy* 2) *scale*))
 (setq *lroll* (* (elt *ljoy* 3) *scale-pitch*))
 (setq *lpitch* (* (elt *ljoy* 4) *scale-pitch*))
 (setq *lyaw* (* (elt *ljoy* 5) *scale-yaw*)) 
 (setq *lbot1* (elt *lbot* 0))
 (setq *lbot2* (elt *lbot* 1))
 
 ;; Right arm control
 (setq *rjoyx* (* (elt *rjoy* 0) *scale*))
 (setq *rjoyy* (* (elt *rjoy* 1) *scale*))
 (setq *rjoyz* (* (elt *rjoy* 2) *scale*))
 (setq *rroll* (* (elt *rjoy* 3) *scale-pitch*))
 (setq *rpitch* (* (elt *rjoy* 4) *scale-pitch*))
 (setq *ryaw* (* (elt *rjoy* 5) *scale-yaw*)) 
 (setq *rbot1* (elt *rbot* 0))
 (setq *rbot2* (elt *rbot* 1))
 
 (if (or (not (= *ljoyx* 0)) (not (= *ljoyy* 0)) (not (= *ljoyz* 0)) 
         (not (= *lroll* 0)) (not (= *lpitch* 0)))
     (progn
       (setq *larm-coords* (send (send *nextage* :larm :end-coords) :copy-worldcoords))
       (send *larm-coords* :translate (float-vector *ljoyz* *ljoyy* (- 0 *ljoyx*)))
       (send *larm-coords* :rotate *lpitch* :y)
       (send *larm-coords* :rotate *lroll* :x)
       (send *nextage* :larm :inverse-kinematics *larm-coords* :rotation-axis t :stop 3 :revert-if-fail nil)))
 
 (if (or (not (= *rjoyx* 0)) (not (= *rjoyy* 0)) (not (= *rjoyz* 0)) 
         (not (= *rroll* 0)) (not (= *rpitch* 0)))
     (progn
       (setq *rarm-coords* (send (send *nextage* :rarm :end-coords) :copy-worldcoords))
       (send *rarm-coords* :translate (float-vector *rjoyz* *rjoyy* (- 0 *rjoyx*)))
       (send *rarm-coords* :rotate *rpitch* :y)
       (send *rarm-coords* :rotate *rroll* :x)
       (send *nextage* :rarm :inverse-kinematics *rarm-coords* :rotation-axis t :stop 3 :revert-if-fail nil)))
 
 ;; Send angle vector for both arms
 (send *ri* :angle-vector (send *nextage* :angle-vector) 500)
 
 ;; Left forceps control
 (if (or (= *lbot1* 1) (= *lbot2* 1))
     (progn
       (if (> (abs *lyaw*) 0)
           (if (> *lyaw* 0)
               (setq *l-forceps-rotation* (+ *l-forceps-rotation* 0.2))
             (setq *l-forceps-rotation* (- *l-forceps-rotation* 0.2))))
       ;;(setq *l-forceps-rotation* (pi_cycle *l-forceps-rotation*))
       
       (if (= *lbot1* 1)
           (progn
             (send *ri* :lforceps :grasp t :rotate *l-forceps-rotation*)
             (setq *l-grasp-position* t)))
       
       (if (= *lbot2* 1)
           (progn
             (send *ri* :lforceps :grasp nil :rotate *l-forceps-rotation*)
             (setq *l-grasp-position* nil)))
       
       (if (and (= *lbot1* 1) (= *lbot2* 1))
	   (send *ri* :lforceps :grasp *l-grasp-position* :rotate *l-forceps-rotation*))))
 
 ;; Right forceps control
 (if (or (= *rbot1* 1) (= *rbot2* 1))
     (progn
       (if (> (abs *ryaw*) 0)
           (if (> *ryaw* 0)
               (setq *r-forceps-rotation* (+ *r-forceps-rotation* 0.2))
             (setq *r-forceps-rotation* (- *r-forceps-rotation* 0.2))))

       ;;(setq *r-forceps-rotation* (pi_cycle *r-forceps-rotation*))
       
       (if (= *rbot1* 1)
           (progn
             (send *ri* :rforceps :grasp t :rotate *r-forceps-rotation*)
             (setq *r-grasp-position* t)))
       
       (if (= *rbot2* 1)
           (progn
             (send *ri* :rforceps :grasp nil :rotate *r-forceps-rotation*)
             (setq *r-grasp-position* nil)))
       
       (if (and (= *rbot1* 1) (= *rbot2* 1))
           (send *ri* :rforceps :grasp *r-grasp-position* :rotate *r-forceps-rotation*))))
 
 (send *irtviewer* :draw-objects)
 ;;(send *ri* :wait-interpolation)
 (ros::duration-sleep 0.5)
 (ros::spin-once))