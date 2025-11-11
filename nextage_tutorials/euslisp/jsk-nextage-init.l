#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(nextage-init)

(setq *robot* *nextage*)
(objects (list *robot*))
(send *robot* :reset-pose)
(send *robot* :head :angle-vector #f(0 60))
(send *robot* :larm :move-end-pos #f(0 0 100))
(send *robot* :rarm :move-end-pos #f(0 0 100))
(send *ri* :angle-vector (send *nextage* :angle-vector) 2000)
(send *ri* :wait-interpolation)
(send *ri* :close-holder :rarm)
(send *ri* :open-forceps :larm)
(send *irtviewer* :draw-objects)
