#!/usr/bin/env roseus

(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(nextage-init)
(send *nextage* :angle-vector #f(0.0 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 0.0))
(send *ri* :angle-vector (send *nextage* :angle-vector) 15000)

;;  TODO : Slave側でhrpsysとopenrtmがbuildできていないので，(send *ri* :servo-off) するためのserviceに送れない．
