#!/usr/bin/env roseus
;; Load packages
(ros::load-ros-manifest "roseus")
(ros::roseus "nextage-phantom")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "omni_msgs")
(load "models/arrow-object.l")
(require "package://nextage_tutorials/euslisp/nextage-interface.l")
(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(require "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")
(require "package://dynamixel_detachable_hand/euslisp/rhand-interface.l")
;; Parameters
(setq *use-torso* nil)
(setq *use-hand* nil)
(setq *base-offset-l* (float-vector 350 100 0))
(setq *base-offset-r* (float-vector 350 -100 0))
(setq *send-time* 200)
(setq *ratio* 0.8)
(ros::rate 50)

(defun initialize-variables()
  (setq *lpos* nil)
  (setq *lrot* nil)
  (setq *ltf* nil)
  (setq *lcoords* nil)
  (setq *lclose* nil)
  (setq *llocked* nil)
  (setq *rpos* nil)
  (setq *rrot* nil)
  (setq *rtf* nil)
  (setq *rcoords* nil)
  (setq *rclose* nil)
  (setq *rlocked* nil)
  )

(defun check-work-limit()
  )

(defun left-phantom-cb (msg)
  (setq *lpos* (send msg :pose :position))
  (setq *lrot* (send msg :pose :orientation))
;;  (setq *ltf* (ros::tf-pose->coords (send msg :pose)))
;;  (setq *lcoords* (make-coords :rot (copy-object (send *ltf* :rot))))
;;  (send *lcoords* :rotate pi/2 :x :world) 
;;  (send *lcoords* :locate (send *ltf* :worldpos) :world)
  (setq *lclose* (send msg :close_gripper))
  (setq *llocked* (send msg :locked))
  )

(defun right-phantom-cb (msg)
  (setq *rpos* (send msg :pose :position))
  (setq *rrot* (send msg :pose :orientation))
;;  (setq *rtf* (ros::tf-pose->coords (send msg :pose)))
;;  (setq *rcoords* (make-coords :rot (copy-object (send *rtf* :rot))))
;;  (send *rcoords* :rotate pi/2 :x :world) 
;;  (send *rcoords* :locate (send *rtf* :worldpos) :world)
  (setq *rclose* (send msg :close_gripper))
  (setq *rlocked* (send msg :locked))
  )


;; Initialize
(ros::rate 10)
(ros::subscribe "/left_device/phantom/state" omni_msgs::OmniState #'left-phantom-cb)
(ros::subscribe "/right_device/phantom/state" omni_msgs::OmniState #'right-phantom-cb)

(initialize-variables)
(nextage-init)
(lhand-init)
(rhand-init)
(setq *lc* (make-cube 10 10 10)) ;; left-center
(send *lc* :translate *base-offset-l*)
(send *lc* :set-color :yellow)
(setq *rc* (make-cube 10 10 10)) ;; right-center
(send *rc* :translate *base-offset-r*)
(send *rc* :set-color :yellow)
(setq *robot* *nextage*)
(setq *la-target* (arrow))
(setq *la-end* (arrow))
(setq *ra-target* (arrow))
(setq *ra-end* (arrow))
(objects (list *nextage* *lc* *rc* *la-target* *la-end*))

;; Reset robot
(send *nextage* :reset-pose)
(send *nextage* :head :angle-vector #f(0 50))
(send *ri* :angle-vector (send *nextage* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; Constants
(setq R_map (matrix (float-vector 0 -1 0)
                    (float-vector 1 0 0)
                    (float-vector 0 0 1)))

;; Main
(do-until-key
 (ros::spin-once)
 
 (setq ik-success t)
 
 (when (and *lpos* *lrot* (not *llocked*))
   (setq *lx* (+ (* (send *lpos* :y) *ratio*) (elt *base-offset-l* 0)))
   (setq *ly* (+ (* (- 0 (send *lpos* :x)) *ratio*) (elt *base-offset-l* 1)))
   (setq *lz* (+ (* (send *lpos* :z) *ratio*) (elt *base-offset-l* 2)))
   (setq lR_src (quaternion2matrix (float-vector (send *lrot* :w)
						 (send *lrot* :x)
						 (send *lrot* :y)
						 (send *lrot* :z))))
   (setq lR_dst (m* R_map lR_src))
   (setq *larm-target* (make-coords :pos (float-vector *lx* *ly* *lz*) :rot lR_dst))
   (unless (send *nextage* :larm :inverse-kinematics *larm-target* :use-torso *use-torso* :rotation-axis t :stop 10 :revert-if-fail t)
     (setq ik-success nil)))
 
 (when (and *rpos* *rrot* (not *rlocked*))
   (setq *rx* (+ (* (send *rpos* :y) *ratio*) (elt *base-offset-r* 0)))
   (setq *ry* (+ (* (- 0 (send *rpos* :x)) *ratio*) (elt *base-offset-r* 1)))
   (setq *rz* (+ (* (send *rpos* :z) *ratio*) (elt *base-offset-r* 2)))
   (setq rR_src (quaternion2matrix (float-vector (send *rrot* :w)
						 (send *rrot* :x)
						 (send *rrot* :y)
						 (send *rrot* :z))))
   (setq rR_dst (m* R_map rR_src))
   (setq *rarm-target* (make-coords :pos (float-vector *rx* *ry* *rz*) :rot rR_dst))
   (unless (send *nextage* :rarm :inverse-kinematics *rarm-target* :use-torso *use-torso* :rotation-axis t :stop 10 :revert-if-fail t)
     (setq ik-success nil)))
 
 (when *lclose* (send *lhand* :close-forceps))
 (unless *lclose* (send *lhand* :open-forceps))
 (when *rclose* (send *rhand* :close-holder))
 (unless *rclose* (send *rhand* :open-holder))
 
 (when (and ik-success (not (nextage-collision-check)))
   (send *ri* :angle-vector (send *nextage* :angle-vector) *send-time*))
 
 (send *irtviewer* :draw-objects)
 (ros::sleep)
 )
#|
Memo: Using the tf conversion method descripted in https://github.com/YUKINA-3252/jsk_hironx_teleop/issues/2, we cannot control the ration of :translate, 
So I don't use it intentionally.
=#
