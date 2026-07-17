(ros::load-ros-package "control_msgs")
(ros::load-ros-package "trajectory_msgs")
(ros::load-ros-package "builtin_interfaces")
(ros::load-ros-package "sensor_msgs")
(require :nextage "package://nextage_tutorials/nextage.l")

(defun shortest-angle (d0 d1)
  "Return the shortest angular displacement from D1 to D0 in degrees."
  (rad2deg (atan2 (sin (deg2rad (- d0 d1)))
                  (cos (deg2rad (- d0 d1))))))

(defclass nextage-interface
  :super object
  :slots (robot controller-action action-client joint-states-topic
          current-angle-vector))

(defmethod nextage-interface
  (:init (&key ((:robot r) nextageopen-robot)
               ((:controller-action ca)
                "/fullbody_controller/follow_joint_trajectory")
               ((:joint-states-topic jst) "/robot/joint_states"))
    (setq robot (if (derivedp r metaclass) (instance r :init) r))
    (setq controller-action ca)
    (setq joint-states-topic jst)
    (setq current-angle-vector (copy-seq (send robot :angle-vector)))
    (setq action-client
          (instance ros::simple-action-client :init controller-action
                    control_msgs::FollowJointTrajectory))
    (ros::subscribe joint-states-topic sensor_msgs::JointState
                    #'send self :joint-state-callback 1)
    self)

  (:robot () robot)

  ;; Compatibility with the commonly used pr2eus robot-interface API.
  (:state () self)
  (:potentio-vector () (copy-seq current-angle-vector))
  (:action-client () action-client)

  (:wait-for-server (&optional timeout)
    (send action-client :wait-for-server timeout))

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
    (let ((point (instance trajectory_msgs::JointTrajectoryPoint :init)))
      (send point :positions
            (map float-vector #'(lambda (degree) (deg2rad degree)) angles))
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
    (unless (send action-client :wait-for-server 10.0)
      (error "FollowJointTrajectory server is unavailable: ~A"
             controller-action))
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
        (send action-client :send-goal goal))))

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
      (mapc #'(lambda (joint angle)
                (let ((index (position (send joint :name) names :test #'string=)))
                  (when index
                    ;; Simulated sensor noise can exceed a hard joint limit by
                    ;; a fraction of a degree.  Keep the model valid without
                    ;; discarding the raw value returned by :potentio-vector.
                    (send joint :joint-angle
                          (max (send joint :min-angle)
                               (min angle (send joint :max-angle)))))))
            joints measured)
      (setq current-angle-vector measured)))

  (:interpolatingp ()
    (send action-client :spin-once)
    (member (send action-client :get-state)
            (list ros::*action-goal-state-accepted*
                  ros::*action-goal-state-executing*
                  ros::*action-goal-state-canceling*)))

  (:wait-interpolation ()
    (unless (send action-client :wait-for-result)
      (return-from :wait-interpolation nil))
    (let ((result (send action-client :get-result)))
      (and (= (send action-client :get-state)
              ros::*action-goal-state-succeeded*)
           (= (send result :error_code) 0)))))

(defun nextage-init ()
  (unless (ros::ok) (ros::roseus "nextage_interface"))
  (unless (boundp '*nextage*)
    (setq *nextage* (instance nextageopen-robot :init)))
  (unless (boundp '*ri*)
    (setq *ri* (instance nextage-interface :init :robot *nextage*)))
  *ri*)
