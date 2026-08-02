(require "package://nextage_tutorials/euslisp/jsk-nextage-interface.l")

(defun nextage-valid-collision-pair-p (pair)
  (and (send (car pair) :faces)
       (send (cdr pair) :faces)))

(defmethod NextageOpen-robot
  (:self-collision-check
   (&rest args
    &key (mode :all)
         (pairs (send self :collision-check-pairs))
         (collision-func 'collision-check)
    &allow-other-keys)
    (let ((ret (send-super :self-collision-check
                           :mode mode
                           :pairs (remove-if
                                   #'(lambda (pair)
                                       (not (nextage-valid-collision-pair-p pair)))
                                   pairs)
                           :collision-func collision-func)))
      (when ret
        (setq ret (remove-if #'(lambda (l)
                                 (or (and (eq (car l) (send self :link "CHEST_JOINT0_Link"))
                                          (eq (cdr l) (send self :link "HEAD_JOINT1_Link")))
                                     (and (eq (cdr l) (send self :link "CHEST_JOINT0_Link"))
                                          (eq (car l) (send self :link "HEAD_JOINT1_Link")))
                                     (and (eq (car l) (send self :link "LARM_JOINT4_Link"))
                                          (eq (cdr l) (send self :link "LARM_JOINT5_Link")))
                                     (and (eq (cdr l) (send self :link "LARM_JOINT4_Link"))
                                          (eq (car l) (send self :link "LARM_JOINT5_Link")))
                                     (and (eq (car l) (send self :link "RARM_JOINT4_Link"))
                                          (eq (cdr l) (send self :link "RARM_JOINT5_Link")))
                                     (and (eq (cdr l) (send self :link "RARM_JOINT4_Link"))
                                          (eq (car l) (send self :link "RARM_JOINT5_Link")))))
                             ret)))
      ret)))
