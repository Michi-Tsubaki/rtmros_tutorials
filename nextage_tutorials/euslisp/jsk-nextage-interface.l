(ros::load-ros-package "control_msgs")
(ros::load-ros-package "trajectory_msgs")
(ros::load-ros-package "builtin_interfaces")
(ros::load-ros-package "sensor_msgs")
(require :nextage "package://nextage_tutorials/jsk-nextage.l")

(defun shortest-angle (d0 d1)
  "Return the shortest angular displacement from D1 to D0 in degrees."
  (rad2deg (atan2 (sin (deg2rad (- d0 d1)))
                  (cos (deg2rad (- d0 d1))))))

(defun nextage-joint-list->joint-state (joint-list)
  (let ((msg (instance sensor_msgs::JointState :init)))
    (send msg :name (send-all joint-list :name))
    (send msg :position
          (map float-vector
               #'(lambda (joint)
                   (if (derivedp joint linear-joint)
                       (* 0.001 (send joint :joint-angle))
                     (deg2rad (send joint :joint-angle))))
               joint-list))
    (send msg :velocity (instantiate float-vector (length joint-list)))
    (send msg :effort (instantiate float-vector (length joint-list)))
    msg))

(defclass nextage-controller-action-client
  :super ros::simple-action-client
  :slots (ri angle-vector-sequence timer-sequence current-time
          previous-angle-vector))

(defmethod nextage-controller-action-client
  (:init (r &rest args)
    (setq ri r)
    (setq angle-vector-sequence nil)
    (setq timer-sequence nil)
    (setq current-time 0)
    (setq previous-angle-vector nil)
    (send-super* :init args))

  (:clear-angle-vector-simulation (&optional previous)
    (setq angle-vector-sequence nil)
    (setq timer-sequence nil)
    (setq current-time 0)
    (setq previous-angle-vector (if previous (copy-seq previous) nil)))

  (:push-angle-vector-simulation (av tm &optional previous)
    (when previous
      (setq previous-angle-vector (copy-seq previous)))
    (setq angle-vector-sequence
          (append angle-vector-sequence (list (copy-seq av))))
    (setq timer-sequence (append timer-sequence (list tm)))
    av)

  (:pop-angle-vector-simulation ()
    (let ((av (car angle-vector-sequence))
          (tm (car timer-sequence))
          scale-av next-av)
      (when (and av tm)
        (unless previous-angle-vector
          (setq previous-angle-vector (send ri :potentio-vector)))
        (setq scale-av (send ri :sub-angle-vector av previous-angle-vector))
        (setq next-av (v+ previous-angle-vector
                          (scale (/ current-time tm) scale-av)))
        (incf current-time 100.0)
        (when (>= current-time tm)
          (pop timer-sequence)
          (pop angle-vector-sequence)
          (setq next-av (v+ previous-angle-vector scale-av))
          (setq previous-angle-vector (copy-seq next-av))
          (setq current-time 0))
        next-av)))

  (:interpolatingp ()
    (if (send ri :simulation-modep)
        (not (null timer-sequence))
      (member (send self :get-state)
              (list ros::*action-goal-state-accepted*
                    ros::*action-goal-state-executing*
                    ros::*action-goal-state-canceling*)))))

(defclass nextage-interface
  :super object
  :slots (robot controller-action action-client joint-states-topic
          pub-joint-states-topic current-angle-vector execution-mode
          fallback-warned-p objects viewer simulation-default-look-all-p
          simulation-started-p))

(defmethod nextage-interface
  (:init (&key ((:robot r) nextageopen-robot)
               ((:controller-action ca)
                "/fullbody_controller/follow_joint_trajectory_action")
               ((:joint-states-topic jst) "/robot/joint_states")
               ((:publish-joint-states-topic pjst) nil)
               ((:objects objs) nil)
               ((:execution-mode em) :auto)
               ((:simulation-look-all sim-look-all) t))
    (unless (ros::ok)
      (ros::roseus "nextage_interface"))
    (setq robot (if (derivedp r metaclass) (instance r :init) r))
    (setq controller-action ca)
    (setq joint-states-topic jst)
    (setq pub-joint-states-topic pjst)
    (setq current-angle-vector (copy-seq (send robot :angle-vector)))
    (setq execution-mode em)
    (setq fallback-warned-p nil)
    (setq objects objs)
    (setq viewer nil)
    (setq simulation-default-look-all-p sim-look-all)
    (setq simulation-started-p nil)
    (setq action-client
          (instance nextage-controller-action-client :init self controller-action
                    control_msgs::FollowJointTrajectory))
    (ros::subscribe joint-states-topic sensor_msgs::JointState
                    #'send self :joint-state-callback 1)
    (when (eq execution-mode :kinematic-simulator)
      (send self :start-kinematic-simulator))
    self)

  (:robot () robot)
  (:objects (&optional objs)
    (when objs
      (setq objects objs)
      (when viewer
        (send viewer :objects (cons robot objects))
        (send self :draw-objects)))
    objects)

  ;; Compatibility with the commonly used pr2eus robot-interface API.
  (:state () self)
  (:potentio-vector () (copy-seq current-angle-vector))
  (:action-client () action-client)
  (:execution-mode () execution-mode)
  (:simulation-modep () (eq execution-mode :kinematic-simulator))

  (:wait-for-server (&optional timeout)
    (send action-client :wait-for-server timeout))

  (:use-kinematic-simulator ()
    (send self :start-kinematic-simulator))

  (:joint-states-publish-topic ()
    (or pub-joint-states-topic joint-states-topic))

  (:install-kinematic-simulator-timer ()
    (setq *top-selector-interval* 0.01)
    (setq *timer-job*
          (remove-if
           #'(lambda (job)
               (when (and (listp job) (>= (length job) 3))
                 (let ((body (caddr job)))
                   (and (listp body)
                        (eq (car body) 'send)
                        (eq (cadr body) self)
                        (eq (caddr body) :kinematic-simulator-callback)))))
           *timer-job*))
    (pushnew `(lambda () (send ,self :kinematic-simulator-callback))
             *timer-job*
             :test #'equal))

  (:start-kinematic-simulator ()
    (setq execution-mode :kinematic-simulator)
    (setq current-angle-vector (copy-seq (send robot :angle-vector)))
    (unless simulation-started-p
      (ros::ros-warn "Starting 'Kinematics Simulator' for ~A"
                     (send robot :name))
      (let ((old-viewer user::*viewer*))
        (when (and x::*display* (> x::*display* 0))
          (setq viewer (get (geo::find-viewer (send robot :name)) :pickviewer))
          (unless viewer
            (setq viewer
                  (instance x::irtviewer :create
                            :title (format nil "~A Kinematics Simulator"
                                           (send robot :name))
                            :view-name (send robot :name)
                            :draw-floor t)))
          (send viewer :objects (cons robot objects))
          (send self :draw-objects)
          (when old-viewer
            (setq user::*viewer* old-viewer))))
      (ros::advertise (send self :joint-states-publish-topic)
                      sensor_msgs::JointState)
      (send self :install-kinematic-simulator-timer)
      (send action-client :clear-angle-vector-simulation current-angle-vector)
      (send self :publish-joint-state)
      (setq simulation-started-p t))
    self)

  (:ensure-controller-or-simulator (&optional (timeout 10.0))
    (cond
      ((eq execution-mode :kinematic-simulator)
       nil)
      ((send action-client :wait-for-server timeout)
       (setq execution-mode :controller)
       t)
      ((eq execution-mode :controller)
       (error "FollowJointTrajectory server is unavailable: ~A"
              controller-action))
      (t
       (send self :use-kinematic-simulator)
       (unless fallback-warned-p
         (ros::ros-warn
          "FollowJointTrajectory server is unavailable: ~A; using kinematic simulator"
          controller-action)
         (setq fallback-warned-p t))
       nil)))

  (:draw-objects (&key look-all)
    (when viewer
      (send viewer :objects (cons robot objects))
      (when (or simulation-default-look-all-p look-all)
        (send viewer :look-all
              (send (geo::make-bounding-box
                     (flatten (send-all (x::draw-things robot) :vertices)))
                    :grow 0.3)))
      (send viewer :draw-objects)))

  (:publish-joint-state (&optional (joint-list (send robot :joint-list)))
    (let ((msg (nextage-joint-list->joint-state joint-list))
          (now (ros::time-now)))
      (send msg :header :stamp :sec (send now :sec))
      (send msg :header :stamp :nanosec (send now :nsec))
      (when (send self :simulation-modep)
        (ros::publish (send self :joint-states-publish-topic) msg))
      msg))

  (:queue-angle-vector-simulation (sequence times)
    (send action-client :clear-angle-vector-simulation current-angle-vector)
    (mapc #'(lambda (angles tm)
              (send action-client :push-angle-vector-simulation angles tm))
          sequence times)
    (send self :kinematic-simulator-callback)
    t)

  (:kinematic-simulator-callback ()
    (when (send self :simulation-modep)
      (let ((av (send action-client :pop-angle-vector-simulation)))
        (when av
          (send robot :angle-vector av)
          (setq current-angle-vector (copy-seq av))))
      (send self :publish-joint-state)
      (send self :draw-objects)))

  (:joint-names ()
    (mapcar #'(lambda (joint) (send joint :name)) (send robot :joint-list)))

  (:make-duration (tm)
    (unless (and (numberp tm) (> tm 0))
      (error "trajectory duration must be positive milliseconds: ~A" tm))
    (let* ((nanoseconds (round (* tm 1000000)))
           (duration (instance builtin_interfaces::Duration :init)))
      (send duration :sec (floor (/ nanoseconds 1000000000)))
      (send duration :nanosec (mod nanoseconds 1000000000))
      duration))

  (:make-point (angles tm)
    (let ((point (instance trajectory_msgs::JointTrajectoryPoint :init))
          (zeros (instantiate float-vector (length angles))))
      (send point :positions
            (map float-vector #'(lambda (degree) (deg2rad degree)) angles))
      (send point :velocities zeros)
      (send point :time_from_start (send self :make-duration tm))
      point))

  (:angle-vector (angles &optional (tm 1000))
    "Send ANGLES (degrees) to the ROS 2 full-body trajectory action."
    (send self :angle-vector-sequence (list angles) (list tm)))

  (:angle-vector-sequence (sequence &optional times)
    "Send a degree angle-vector sequence. TIMES contains segment msec."
    (unless times (setq times (make-list (length sequence) :initial-element 1000)))
    (unless (= (length sequence) (length times))
      (error "angle-vector-sequence and times must have the same length"))
    (when (null sequence)
      (error "angle-vector-sequence must not be empty"))
    (let ((joint-count (length (send self :joint-names))))
      (dolist (angles sequence)
        (unless (= (length angles) joint-count)
          (error "expected ~A joint angles, got ~A"
                 joint-count (length angles)))))
    (dolist (tm times)
      (unless (and (numberp tm) (> tm 0))
        (error "trajectory duration must be positive milliseconds: ~A" tm)))
    (if (not (send self :ensure-controller-or-simulator 10.0))
        (send self :queue-angle-vector-simulation sequence times)
      (let ((trajectory (instance trajectory_msgs::JointTrajectory :init))
            (elapsed 0)
            points)
        (send trajectory :joint_names (send self :joint-names))
        (mapc #'(lambda (angles tm)
                  (incf elapsed tm)
                  (setq points (append points (list (send self :make-point angles elapsed)))))
              sequence times)
        (send trajectory :points points)
        (let ((goal (instance control_msgs::FollowJointTrajectory_Goal :init)))
          (send goal :trajectory trajectory)
          (send action-client :send-goal goal)))))

  (:sub-angle-vector (angles reference)
    (map float-vector #'shortest-angle angles reference))

  (:joint-state-callback (msg)
    (let ((names (send msg :name))
          (positions (send msg :position))
          (joints (send robot :joint-list))
          measured)
      (setq measured
            (map float-vector
                 #'(lambda (joint old-angle)
                     (let ((index (position (send joint :name) names
                                            :test #'string=)))
                       (if index (rad2deg (elt positions index)) old-angle)))
                 joints current-angle-vector))
      (map nil #'(lambda (joint angle)
                   (let ((index
                          (position (send joint :name) names :test #'string=)))
                     (when index
                       ;; Simulated sensor noise can exceed a hard joint limit
                       ;; by a fraction of a degree.  Keep the model valid
                       ;; without discarding the raw :potentio-vector value.
                       (send joint :joint-angle
                             (max (send joint :min-angle)
                                  (min angle (send joint :max-angle)))))))
           joints measured)
      (setq current-angle-vector measured)))

  (:interpolatingp ()
    (if (eq execution-mode :kinematic-simulator)
        (send action-client :interpolatingp)
      (progn
        (send action-client :spin-once)
        (send action-client :interpolatingp))))

  (:wait-interpolation ()
    (when (eq execution-mode :kinematic-simulator)
      (while (send self :interpolatingp)
        (send self :kinematic-simulator-callback))
      (return-from :wait-interpolation t))
    (unless (send action-client :wait-for-result)
      (return-from :wait-interpolation nil))
    (let ((result (send action-client :get-result)))
      (and (= (send action-client :get-state)
              ros::*action-goal-state-succeeded*)
           (= (send result :error_code) 0)))))

(defun call-with-stdio-to-devnull (thunk)
  "Run THUNK with fd 1/2 (stdout/stderr) redirected to /dev/null.
Used to silence jskeus's per-triangle \"PQP_AddTri\" warnings that flood
the console when eus is built without PQP support."
  (let* ((devnull (open "/dev/null" :direction :output :if-exists :append))
         (devnull-fd (send devnull :outfd))
         (saved-out (unix:dup 1))
         (saved-err (unix:dup 2)))
    (unwind-protect
        (progn (unix:dup2 devnull-fd 1)
               (unix:dup2 devnull-fd 2)
               (funcall thunk))
      (unix:dup2 saved-out 1)
      (unix:dup2 saved-err 2)
      (unix::uclose saved-out)
      (unix::uclose saved-err)
      (close devnull))))

(defun nextage-init
  (&key ((:controller-action ca)
         "/fullbody_controller/follow_joint_trajectory_action")
        ((:joint-states-topic jst) "/robot/joint_states")
        ((:publish-joint-states-topic pjst) nil)
        ((:objects objs) nil)
        ((:execution-mode em) :auto)
        ((:simulation-look-all sim-look-all) t))
  (unless (ros::ok) (ros::roseus "nextage_interface"))
  (unless (boundp '*nextage*)
    (setq *nextage* (call-with-stdio-to-devnull
                      #'(lambda () (instance nextageopen-robot :init)))))
  (setq *ri* (instance nextage-interface :init
                       :robot *nextage*
                       :controller-action ca
                       :joint-states-topic jst
                       :publish-joint-states-topic pjst
                       :objects objs
                       :execution-mode em
                       :simulation-look-all sim-look-all))
  *ri*)
