#!/usr/bin/env roseus


(ros::roseus "jsk_nextage_dynamixel_teleop")
(ros::load-ros-package "touch_msgs")

(load "models/arrow-object.l")
(require "package://roseus/euslisp/actionlib2.l")
(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(require "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")
(require "package://dynamixel_detachable_hand/euslisp/rhand-interface.l")
(require "package://dynamixel_detachable_hand/euslisp/tool-change.l")

(setq *nextage-teleop-left-touch-state-topic* "/left_ft/state")
(setq *nextage-teleop-right-touch-state-topic* "/right_ft/state")
(setq *nextage-teleop-base-offset-l* #f(500 80 -100))
(setq *nextage-teleop-base-offset-r* #f(500 -80 -100))
(setq *nextage-teleop-auto-offset* t)
;; Operator faces the robot.  Keep the same mapping for position and
;; orientation: touch +x -> robot +y, touch +y -> robot -x, touch +z -> robot +z.
(setq *nextage-teleop-touch-frame-map*
      #2f((0.0 -1.0 0.0)
          (1.0 0.0 0.0)
          (0.0 0.0 1.0)))
(setq *nextage-teleop-target-z-rotation* (deg2rad 90))
(setq *nextage-teleop-gripper-base-offset*
  #f(31.905903 68.529998 36.32414))
(setq *nextage-teleop-gripper-tip-offset* #f(0 0 67.43))
(setq *nextage-teleop-position-scale* 2.0)
(setq *nextage-teleop-rate* 20.0)
(setq *nextage-teleop-command-rate* 20.0)
(setq *nextage-teleop-send-time* 200)
(setq *nextage-teleop-command-transport* :action)
(setq *nextage-teleop-command-topic* "/fullbody_controller/command")
(setq *nextage-teleop-preempt-active-goal* t)
(setq *nextage-teleop-initial-pose-time* 5000)
(setq *nextage-teleop-use-torso* nil)
(setq *nextage-teleop-debug-mode* t)
(setq *nextage-teleop-check-self-collision* t)
(setq *nextage-teleop-debug-print-period* 20)
(setq *nextage-teleop-left-tool* :gripper)
(setq *nextage-teleop-right-tool* :gripper)
(setq *nextage-teleop-launch-hand-controller* t)
(setq *nextage-teleop-hand-launch-command*
  "nohup bash -lc \"source ~/colcon_ws/install/setup.bash && exec ros2 launch dynamixel_detachable_hand dual_hand.launch.xml left_tool:=gripper right_tool:=gripper\" > /tmp/nextage_teleop_dynamixel_hand.log 2>&1 &")
(setq *nextage-teleop-hand-launch-wait* 5.0)
(setq *nextage-teleop-hand-server-timeout* 15.0)
(setq *nextage-teleop-gripper-open-time* 0.25)

(defvar *ltarget-arrow* nil)
(defvar *lpinch-arrow* nil)
(defvar *rtarget-arrow* nil)
(defvar *rpinch-arrow* nil)
(defvar *nextage-teleop-viewer-objects* nil)
(defvar *nextage-teleop-left-calibration* nil)
(defvar *nextage-teleop-right-calibration* nil)
(defvar *nextage-teleop-left-pinch-coords* nil)
(defvar *nextage-teleop-right-pinch-coords* nil)
(defvar *nextage-teleop-hand-launch-started* nil)
(defvar *nextage-teleop-last-arm-command-loop* nil)
(defvar *nextage-teleop-command-publisher-advertised* nil)

(defun nt-dissoc-coords (coords)
  (when coords
    (let ((parent (send coords :parent)))
      (when parent
        (send parent :dissoc coords)))))

(defun nt-clear-pinch ()
  (nt-dissoc-coords *nextage-teleop-left-pinch-coords*)
  (nt-dissoc-coords *nextage-teleop-right-pinch-coords*)
  (setq *nextage-teleop-left-pinch-coords* nil)
  (setq *nextage-teleop-right-pinch-coords* nil))

(defun nt-init-state ()
  (setq *nextage-teleop-loop-count* 0)
  (setq *nextage-teleop-lpos* nil)
  (setq *nextage-teleop-lrot* nil)
  (setq *nextage-teleop-lclose* nil)
  (setq *nextage-teleop-llocked* nil)
  (setq *nextage-teleop-rpos* nil)
  (setq *nextage-teleop-rrot* nil)
  (setq *nextage-teleop-rclose* nil)
  (setq *nextage-teleop-rlocked* nil)
  (setq *nextage-teleop-prev-lclose* nil)
  (setq *nextage-teleop-prev-rclose* nil)
  (setq *nextage-teleop-left-received* nil)
  (setq *nextage-teleop-right-received* nil)
  (setq *nextage-teleop-left-calibration* nil)
  (setq *nextage-teleop-right-calibration* nil)
  (setq *nextage-teleop-last-arm-command-loop* nil)
  (setq *nextage-teleop-command-publisher-advertised* nil)
  (nt-clear-pinch))

(defun nt-left-cb (msg)
  (setq *nextage-teleop-lpos* (send msg :pose :position))
  (setq *nextage-teleop-lrot* (send msg :pose :orientation))
  (setq *nextage-teleop-lclose* (send msg :close_gripper))
  (setq *nextage-teleop-llocked* (send msg :locked))
  (setq *nextage-teleop-left-received* t))

(defun nt-right-cb (msg)
  (setq *nextage-teleop-rpos* (send msg :pose :position))
  (setq *nextage-teleop-rrot* (send msg :pose :orientation))
  (setq *nextage-teleop-rclose* (send msg :close_gripper))
  (setq *nextage-teleop-rlocked* (send msg :locked))
  (setq *nextage-teleop-right-received* t))

(defun nt-work-ok (robot)
  (let ((lh (elt (send (send (nt-pinch-coords robot :larm)
                             :copy-worldcoords) :pos) 2))
        (rh (elt (send (send (nt-pinch-coords robot :rarm)
                             :copy-worldcoords) :pos) 2)))
    (and (> lh -200) (> rh -200)))
  )

(defun nt-collision-names (pairs)
  (mapcar #'(lambda (pair)
              (list (send (car pair) :name)
                    (send (cdr pair) :name)))
          pairs))

(defun nt-debug-status
    (left-ready right-ready left-active right-active left-ik right-ik
     collision-pairs work-limit-ok command-busy command-due command-sent)
  (when (and *nextage-teleop-debug-mode*
             (> *nextage-teleop-debug-print-period* 0)
             (= (mod *nextage-teleop-loop-count*
                     *nextage-teleop-debug-print-period*)
                0))
    (ros::ros-info
     "NEXTAGE teleop status: rx(L/R)=~A/~A locked(L/R)=~A/~A active(L/R)=~A/~A ik(L/R)=~A/~A collision=~A work=~A busy=~A due=~A sent=~A"
     left-ready right-ready
     *nextage-teleop-llocked* *nextage-teleop-rlocked*
     left-active right-active
     left-ik right-ik
     (if *nextage-teleop-check-self-collision*
         (nt-collision-names collision-pairs)
       :skipped)
     work-limit-ok command-busy command-due command-sent)))

(defun nt-move-object (obj coords)
  (when (and obj coords)
    (send obj :move-to (send coords :copy-worldcoords) :world)))

(defun nt-target-arrow (arm target)
  (case (detachable-tool-normalize-arm arm)
    (:larm (nt-move-object *ltarget-arrow* target))
    (:rarm (nt-move-object *rtarget-arrow* target))))

(defun nt-pinch-parent (robot arm)
  (send robot
        (case (detachable-tool-normalize-arm arm)
          (:larm :LARM_NEJINEJI_Link_lk)
          (:rarm :RARM_NEJINEJI_Link_lk))))

(defun nt-new-pinch (parent name)
  (let ((coords (make-cascoords :name name)))
    (send parent :assoc coords)
    (send coords :newcoords (make-coords))
    ;; Fixed joints from urdf/JSKNextageOpenGripper.urdf:
    ;; *_NEJINEJI_TO_GRIPPER + *hand_to_tip.
    (send coords :translate *nextage-teleop-gripper-base-offset*)
    (send coords :rotate (deg2rad -90) :x :local)
    (send coords :translate *nextage-teleop-gripper-tip-offset*)
    coords))

(defun nt-pinch-coords (robot arm)
  (let* ((a (detachable-tool-normalize-arm arm))
         (parent (nt-pinch-parent robot a))
         coords)
    (setq coords
          (case a
            (:larm *nextage-teleop-left-pinch-coords*)
            (:rarm *nextage-teleop-right-pinch-coords*)))
    (unless (and coords (eq (send coords :parent) parent))
      (when coords
        (nt-dissoc-coords coords))
      (setq coords
            (nt-new-pinch parent
                          (case a
                            (:larm :larm-gripper-pinch)
                            (:rarm :rarm-gripper-pinch))))
      (case a
        (:larm (setq *nextage-teleop-left-pinch-coords* coords))
        (:rarm (setq *nextage-teleop-right-pinch-coords* coords))))
    coords))

(defun nt-pinch-arrows (robot)
  (nt-move-object
   *lpinch-arrow* (nt-pinch-coords robot :larm))
  (nt-move-object
   *rpinch-arrow* (nt-pinch-coords robot :rarm)))

(defun nt-debug-objects (robot)
  (nt-pinch-arrows robot))

(defun nt-ensure-debug-viewer ()
  (unless (and (boundp '*irtviewer*) *irtviewer*)
    (make-irtviewer))
  *irtviewer*)

(defun nt-setup-debug (robot)
  (nt-ensure-debug-viewer)
  (setq *ltarget-arrow* (arrow :name "larm-target-arrow"))
  (setq *lpinch-arrow* (arrow :name "larm-pinch-arrow"))
  (setq *rtarget-arrow* (arrow :name "rarm-target-arrow"))
  (setq *rpinch-arrow* (arrow :name "rarm-pinch-arrow"))
  (setq *nextage-teleop-viewer-objects*
        (list *ltarget-arrow* *lpinch-arrow*
              *rtarget-arrow* *rpinch-arrow*))
  (nt-debug-objects robot)
  (objects (cons robot *nextage-teleop-viewer-objects*))
  (when (and (boundp '*ri*) *ri* (find-method *ri* :objects))
    (send *ri* :objects *nextage-teleop-viewer-objects*))
  *nextage-teleop-viewer-objects*)

(defun nt-touch-position (pos scale)
  (scale scale
         (transform *nextage-teleop-touch-frame-map*
                    (float-vector (send pos :x)
                                  (send pos :y)
                                  (send pos :z)))))

(defun nt-touch-rotation (rot)
  (let ((rot-mat
         (quaternion2matrix
          (float-vector (send rot :w)
                        (send rot :x)
                        (send rot :y)
                        (send rot :z)))))
    (m* (m* *nextage-teleop-touch-frame-map* rot-mat)
        (transpose *nextage-teleop-touch-frame-map*))))

(defun nt-touch-coords (pos rot scale)
  (let ((coords (make-coords :pos (nt-touch-position pos scale)
                             :rot (nt-touch-rotation rot))))
    (send coords :rotate *nextage-teleop-target-z-rotation* :z :local)
    coords))

(defun nt-make-offset (robot arm pos rot scale)
  (let* ((touch (nt-touch-coords pos rot scale))
         (tool (send (nt-pinch-coords robot arm) :copy-worldcoords)))
    (list (v- (send tool :pos) (send touch :pos))
          (m* (send tool :rot) (transpose (send touch :rot))))))

(defun nt-apply-offset (target calibration)
  (send target :translate (car calibration) :world)
  (send target :orient (m* (cadr calibration) (send target :rot)) :world)
  target)

(defun nt-target-coords
    (pos rot base-offset scale &optional calibration)
  (let ((target (nt-touch-coords pos rot scale)))
    (if calibration
        (nt-apply-offset target calibration)
      (send target :translate base-offset :world))
    target))

(defun nt-arm-ik (robot arm target)
  (let ((move-target (nt-pinch-coords robot arm)))
    (send robot arm :inverse-kinematics
          target
          :move-target move-target
          :link-list (send robot :link-list
                           (send move-target :parent)
                           (send robot arm :root-link))
          :rotation-axis t
          :stop 10
          :revert-if-fail t)))

(defun nt-open-tool (hand tool)
  (case (detachable-tool-normalize-tool tool)
    (:needle_holder (send hand :open-holder :wait nil :velocity 0.0))
    (:forceps (send hand :open-forceps :wait nil :velocity 0.0))
    (:gripper (send hand :open :wait nil :tm *nextage-teleop-gripper-open-time* :velocity 0.0))
    (t (send hand :open :wait nil :velocity 0.0))))

(defun nt-close-tool (hand tool)
  (case (detachable-tool-normalize-tool tool)
    (:needle_holder (send hand :close-holder :wait nil :velocity 0.0))
    (:forceps (send hand :close-forceps :wait nil :velocity 0.0))
    (:gripper (send hand :close :wait nil :velocity 0.0))
    (t (send hand :close :wait nil :velocity 0.0))))

(defun nt-hand-command
    (close-state previous-state hand tool)
  (when (not (eq close-state previous-state))
    (if close-state
        (nt-close-tool hand tool)
      (nt-open-tool hand tool)))
  close-state)

(defun nt-positive-number-p (value)
  (and (numberp value) (> value 0)))

(defun nt-normalize-command-transport (transport)
  (case transport
    (:action :action)
    (:topic :topic)
    (t (error "command-transport must be :action or :topic: ~A"
              transport))))

(defun nt-command-interval-loops ()
  (unless (nt-positive-number-p *nextage-teleop-command-rate*)
    (error "command-rate must be positive Hz: ~A"
           *nextage-teleop-command-rate*))
  (unless (nt-positive-number-p *nextage-teleop-rate*)
    (error "rate must be positive Hz: ~A"
           *nextage-teleop-rate*))
  (max 1
       (round (/ *nextage-teleop-rate*
                 *nextage-teleop-command-rate*))))

(defun nt-command-due-p ()
  (or (null *nextage-teleop-last-arm-command-loop*)
      (>= (- *nextage-teleop-loop-count*
             *nextage-teleop-last-arm-command-loop*)
          (nt-command-interval-loops))))

(defun nt-command-busy-p ()
  (if (or (eq *nextage-teleop-command-transport* :topic)
          (send *ri* :simulation-modep))
      nil
    (and (send *ri* :interpolatingp) t)))

(defun nt-command-ready-p (command-busy)
  (or (not command-busy)
      *nextage-teleop-preempt-active-goal*))

(defun nt-advertise-command-topic ()
  (when (and (eq *nextage-teleop-command-transport* :topic)
             (not (send *ri* :simulation-modep)))
    (ros::advertise *nextage-teleop-command-topic*
                    trajectory_msgs::JointTrajectory 1)
    (setq *nextage-teleop-command-publisher-advertised* t)
    (ros::ros-info
     "NEXTAGE teleop: publishing streaming arm commands to ~A"
     *nextage-teleop-command-topic*)))

(defun nt-make-trajectory-command (angles tm)
  (let ((trajectory (instance trajectory_msgs::JointTrajectory :init)))
    (send trajectory :joint_names (send *ri* :joint-names))
    (send trajectory :points (list (send *ri* :make-point angles tm)))
    trajectory))

(defun nt-send-arm-command (angles tm)
  (if (and (eq *nextage-teleop-command-transport* :topic)
           (not (send *ri* :simulation-modep)))
      (progn
        (unless *nextage-teleop-command-publisher-advertised*
          (nt-advertise-command-topic))
        (ros::publish *nextage-teleop-command-topic*
                      (nt-make-trajectory-command angles tm)))
    (send *ri* :angle-vector angles tm)))

(defun nt-move-initial-pose (robot tm)
  (when (and (numberp tm) (> tm 0))
    (ros::ros-info
     "NEXTAGE teleop: moving to initial pose in ~A sec"
     (/ tm 1000.0))
    (send robot :reset-pose)
    (send robot :head :angle-vector #f(0 0))
    (send robot :larm :move-end-pos #f(0 0 -50))
    (send robot :rarm :move-end-pos #f(0 0 -50))
    (send *ri* :angle-vector (send robot :angle-vector) tm)
    (send *ri* :wait-interpolation)))

(defun nt-hand-server-status (lhand rhand timeout)
  (let ((left-ready (send lhand :wait-for-server timeout))
        (right-ready (send rhand :wait-for-server timeout)))
    (list left-ready right-ready)))

(defun nt-hand-server-ready-p (status)
  (and (car status) (cadr status)))

(defun nt-launch-hand-controller (launch-command wait-time)
  (ros::ros-info
   "NEXTAGE teleop: launching dynamixel detachable grippers: ~A"
   launch-command)
  (let ((status (unix:system launch-command)))
    (unless (= status 0)
      (error "failed to launch dynamixel detachable hand controller (~A): ~A"
             status launch-command)))
  (setq *nextage-teleop-hand-launch-started* t)
  (when (and wait-time (> wait-time 0.0))
    (ros::ros-info
     "NEXTAGE teleop: waiting ~A sec after launching gripper controllers"
     wait-time)
    (ros::duration-sleep wait-time)))

(defun nt-init-hand-controllers
    (launch-controller launch-command launch-wait server-timeout)
  (setq *nextage-teleop-lhand*
        (instance lhand-interface :init :execution-mode :controller))
  (setq *nextage-teleop-rhand*
        (instance rhand-interface :init :execution-mode :controller))
  (let ((status (nt-hand-server-status
                 *nextage-teleop-lhand* *nextage-teleop-rhand* 0.1)))
    (unless (nt-hand-server-ready-p status)
      (if launch-controller
          (nt-launch-hand-controller launch-command launch-wait)
        (ros::ros-warn
         "NEXTAGE teleop: hand controller launch is disabled; waiting for existing gripper action servers"))))
  (ros::ros-info
   "NEXTAGE teleop: waiting up to ~A sec for gripper action servers"
   server-timeout)
  (let ((status (nt-hand-server-status
                 *nextage-teleop-lhand*
                 *nextage-teleop-rhand*
                 server-timeout)))
    (unless (nt-hand-server-ready-p status)
      (error
       "dynamixel detachable gripper action servers are unavailable: left=~A right=~A timeout=~A log=/tmp/nextage_teleop_dynamixel_hand.log"
       (car status) (cadr status) server-timeout))
    (ros::ros-info
     "NEXTAGE teleop: gripper action servers are ready: left=~A right=~A"
     (car status) (cadr status)))
  (list *nextage-teleop-lhand* *nextage-teleop-rhand*))

(defun nextage-teleop
    (&key
     ((:left-topic left-topic) *nextage-teleop-left-touch-state-topic*)
     ((:right-topic right-topic) *nextage-teleop-right-touch-state-topic*)
     ((:left-tool left-tool) *nextage-teleop-left-tool*)
     ((:right-tool right-tool) *nextage-teleop-right-tool*)
     ((:auto-offset auto-offset) *nextage-teleop-auto-offset*)
     ((:position-scale position-scale) *nextage-teleop-position-scale*)
     ((:send-time send-time) *nextage-teleop-send-time*)
     ((:command-rate command-rate) *nextage-teleop-command-rate*)
     ((:command-transport command-transport)
      *nextage-teleop-command-transport*)
     ((:command-topic command-topic) *nextage-teleop-command-topic*)
     ((:preempt-active-goal preempt-active-goal)
      *nextage-teleop-preempt-active-goal*)
     ((:initial-pose-time initial-pose-time)
      *nextage-teleop-initial-pose-time*)
     ((:rate rate) *nextage-teleop-rate*)
     ((:use-torso use-torso) *nextage-teleop-use-torso*)
     ((:debug debug) *nextage-teleop-debug-mode*)
     ((:check-self-collision check-self-collision)
      *nextage-teleop-check-self-collision*)
     ((:debug-print-period debug-print-period)
      *nextage-teleop-debug-print-period*)
     ((:launch-hand-controller launch-hand-controller)
      *nextage-teleop-launch-hand-controller*)
     ((:hand-launch-command hand-launch-command)
      *nextage-teleop-hand-launch-command*)
     ((:hand-launch-wait hand-launch-wait)
      *nextage-teleop-hand-launch-wait*)
     ((:hand-server-timeout hand-server-timeout)
      *nextage-teleop-hand-server-timeout*)
     ((:execution-mode execution-mode) :controller)
     ((:install-tool-tips install-tool-tips) t)
     ((:controller-action controller-action)
      "/fullbody_controller/follow_joint_trajectory_action")
     ((:joint-states-topic joint-states-topic) "/robot/joint_states"))
  (unless (nt-positive-number-p rate)
    (error "rate must be positive Hz: ~A" rate))
  (unless (nt-positive-number-p command-rate)
    (error "command-rate must be positive Hz: ~A" command-rate))
  (unless (nt-positive-number-p send-time)
    (error "send-time must be positive milliseconds: ~A" send-time))
  (setq command-transport (nt-normalize-command-transport command-transport))
  (when (and (eq command-transport :topic) (not command-topic))
    (error "command-topic must be set when command-transport is :topic"))
  (nt-init-state)
  (setq *nextage-teleop-left-tool* (detachable-tool-normalize-tool left-tool))
  (setq *nextage-teleop-right-tool* (detachable-tool-normalize-tool right-tool))
  (setq *nextage-teleop-auto-offset* auto-offset)
  (setq *nextage-teleop-position-scale* position-scale)
  (setq *nextage-teleop-send-time* send-time)
  (setq *nextage-teleop-command-rate* command-rate)
  (setq *nextage-teleop-command-transport* command-transport)
  (setq *nextage-teleop-command-topic* command-topic)
  (setq *nextage-teleop-preempt-active-goal* preempt-active-goal)
  (setq *nextage-teleop-initial-pose-time* initial-pose-time)
  (setq *nextage-teleop-rate* rate)
  (setq *nextage-teleop-use-torso* use-torso)
  (setq *nextage-teleop-debug-mode* debug)
  (setq *nextage-teleop-check-self-collision* check-self-collision)
  (setq *nextage-teleop-debug-print-period* debug-print-period)
  (setq *nextage-teleop-launch-hand-controller* launch-hand-controller)
  (setq *nextage-teleop-hand-launch-command* hand-launch-command)
  (setq *nextage-teleop-hand-launch-wait* hand-launch-wait)
  (setq *nextage-teleop-hand-server-timeout* hand-server-timeout)

  (ros::subscribe left-topic touch_msgs::TouchState
                  #'nt-left-cb 1)
  (ros::subscribe right-topic touch_msgs::TouchState
                  #'nt-right-cb 1)

  (nextage-init :controller-action controller-action
                :joint-states-topic joint-states-topic
                :execution-mode execution-mode)
  (setq *robot* *nextage*)
  (when (eq execution-mode :controller)
    (send *ri* :ensure-controller-or-simulator 10.0))
  (nt-move-initial-pose *robot* *nextage-teleop-initial-pose-time*)
  (nt-advertise-command-topic)
  (tool-setup-robot *robot* :robot-kind :nextage)
  (when install-tool-tips
    (attach-tool *robot* :larm *nextage-teleop-left-tool*
                 :reload-model nil :restart-control nil :wait nil)
    (attach-tool *robot* :rarm *nextage-teleop-right-tool*
                 :reload-model nil :restart-control nil :wait nil))
  (when *nextage-teleop-debug-mode*
    (nt-setup-debug *robot*)
    (ros::ros-info
     "NEXTAGE teleop debug viewer enabled; execution-mode=~A"
     (send *ri* :execution-mode)))
  (nt-init-hand-controllers
   *nextage-teleop-launch-hand-controller*
   *nextage-teleop-hand-launch-command*
   *nextage-teleop-hand-launch-wait*
   *nextage-teleop-hand-server-timeout*)

  (ros::ros-info "NEXTAGE teleop: left=~A (~A), right=~A (~A), scale=~A, auto-offset=~A"
                 left-topic *nextage-teleop-left-tool*
                 right-topic *nextage-teleop-right-tool*
                 *nextage-teleop-position-scale*
                 *nextage-teleop-auto-offset*)
  (ros::ros-info
   "NEXTAGE teleop timing: loop=~A Hz, command=~A Hz (~A loop interval), horizon=~A ms, transport=~A"
   *nextage-teleop-rate*
   *nextage-teleop-command-rate*
   (nt-command-interval-loops)
   *nextage-teleop-send-time*
   *nextage-teleop-command-transport*)
  (ros::rate *nextage-teleop-rate*)

  (do-until-key
   (ros::spin-once)
   (incf *nextage-teleop-loop-count*)
   (let* ((left-ready (and *nextage-teleop-lpos* *nextage-teleop-lrot* t))
          (right-ready (and *nextage-teleop-rpos* *nextage-teleop-rrot* t))
          (left-active (and left-ready (not *nextage-teleop-llocked*) t))
          (right-active (and right-ready (not *nextage-teleop-rlocked*) t))
          (left-ik (cond ((not left-ready) :waiting)
                         (*nextage-teleop-llocked* :locked)
                         (t :not-run)))
          (right-ik (cond ((not right-ready) :waiting)
                          (*nextage-teleop-rlocked* :locked)
                          (t :not-run)))
          (arm-active (or left-active right-active))
          (ik-success t)
          collision-pairs
          work-limit-ok
          command-busy
          command-due
          command-sent)
     (when (and *nextage-teleop-auto-offset*
                left-ready
                (not *nextage-teleop-left-calibration*))
       (setq *nextage-teleop-left-calibration*
             (nt-make-offset
              *robot* :larm
              *nextage-teleop-lpos*
              *nextage-teleop-lrot*
              *nextage-teleop-position-scale*))
       (ros::ros-info
        "NEXTAGE teleop: calibrated left touch pose to current larm tool pose"))
     (when (and *nextage-teleop-auto-offset*
                right-ready
                (not *nextage-teleop-right-calibration*))
       (setq *nextage-teleop-right-calibration*
             (nt-make-offset
              *robot* :rarm
              *nextage-teleop-rpos*
              *nextage-teleop-rrot*
              *nextage-teleop-position-scale*))
       (ros::ros-info
        "NEXTAGE teleop: calibrated right touch pose to current rarm tool pose"))

     (when left-active
       (let ((target
              (nt-target-coords
               *nextage-teleop-lpos*
               *nextage-teleop-lrot*
               *nextage-teleop-base-offset-l*
               *nextage-teleop-position-scale*
               *nextage-teleop-left-calibration*)))
         (nt-target-arrow :larm target)
         (if (nt-arm-ik *robot* :larm target)
             (setq left-ik :ok)
           (progn
             (setq left-ik :fail)
             (setq ik-success nil)))))

     (when right-active
       (let ((target
              (nt-target-coords
               *nextage-teleop-rpos*
               *nextage-teleop-rrot*
               *nextage-teleop-base-offset-r*
               *nextage-teleop-position-scale*
               *nextage-teleop-right-calibration*)))
         (nt-target-arrow :rarm target)
         (if (nt-arm-ik *robot* :rarm target)
             (setq right-ik :ok)
           (progn
             (setq right-ik :fail)
             (setq ik-success nil)))))

     (when *nextage-teleop-debug-mode*
       (nt-debug-objects *robot*))

     (setq *nextage-teleop-prev-lclose*
           (nt-hand-command
            *nextage-teleop-lclose*
            *nextage-teleop-prev-lclose*
            *nextage-teleop-lhand*
            *nextage-teleop-left-tool*))
     (setq *nextage-teleop-prev-rclose*
           (nt-hand-command
            *nextage-teleop-rclose*
            *nextage-teleop-prev-rclose*
            *nextage-teleop-rhand*
            *nextage-teleop-right-tool*))

     (when *nextage-teleop-check-self-collision*
       (setq collision-pairs (send *robot* :self-collision-check)))
     (setq work-limit-ok (nt-work-ok *robot*))
     (setq command-busy (nt-command-busy-p))
     (setq command-due (nt-command-due-p))
     (setq command-sent nil)
     (when (and ik-success
                arm-active
                (not collision-pairs)
                work-limit-ok
                command-due
                (nt-command-ready-p command-busy))
       (nt-send-arm-command
        (send *robot* :angle-vector)
        *nextage-teleop-send-time*)
       (setq *nextage-teleop-last-arm-command-loop*
             *nextage-teleop-loop-count*)
       (setq command-sent t)
       )
     (when (and *nextage-teleop-debug-mode*
                (boundp '*irtviewer*) *irtviewer*)
       (send *irtviewer* :draw-objects)
       )
     (nt-debug-status
      left-ready right-ready left-active right-active left-ik right-ik
      collision-pairs work-limit-ok command-busy command-due command-sent)
     )
   (ros::duration-sleep (/ 1.0 *nextage-teleop-rate*)))
  ) ;;defun

(defun teleop (&rest args)
  (apply #'nextage-teleop args))
