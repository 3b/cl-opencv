;;; -*- mode: lisp; indent-tabs: nil -*-

(defpackage :cl-opencv
  (:use #:cl #:cffi)
  (:export 
   ;; core - Basic Structures
   ; #:cv-point
   ; #:cv-size
   ; #:cv-rect
   ; #:cv-scalar
   #:cv-matrix
   #:ipl-image
   #:cv-array

   ;; core - Operations on Arrays
   #:release-image


   ;; highgui - User Interface
   #:cv-capture
   #:cv-video-writer

   #:+cvtimg-flip+
   #:+cvtimage-swap-rb+
   #:convert-image
   ; #:create-trackbar
   #:destroy-all-windows
   #:destroy-window
   ; #:get-trackbar-pos
   ; #:init-system
   #:move-window
   #:+window-normal+
   #:+window-autosize+
   #:named-window
   #:resize-window
   ; #:set-mouse-callback
   ; #:set-trackbar-pos
   #:show-image
   #:wait-key

   ;; highgui - Reading and Writing Images and Video
   #:+load-image-unchanged+
   #:+load-image-grayscale+
   #:+load-image-color+
   #:+load-image-anydepth+
   #:+load-image-anycolor+
   #:load-image
   #:load-image-m
   #:save-image
   #:create-camera-capture
   #:create-file-capture
   #:+cap-prop-pos-msec+
   #:+cap-prop-pos-frames+
   #:+cap-prop-pos-avi-ratio+
   #:+cap-prop-frame-width+
   #:+cap-prop-frame-height+
   #:+cap-prop-fps+
   #:+cap-prop-fourcc+
   #:+cap-prop-frame-count+
   #:+cap-prop-format+
   #:+cap-prop-mode+
   #:+cap-prop-brightness+
   #:+cap-prop-contrast+
   #:+cap-prop-saturation+
   #:+cap-prop-hue+
   #:+cap-prop-gain+
   #:+cap-prop-exposure+
   #:+cap-prop-convert-rgb+
   #:+cap-prop-white-balance+
   #:+cap-prop-rectification+
   #:get-capture-property
   #:grab-frame
   #:query-frame
   #:release-capture
   #:retrieve-frame
   #:set-capture-property
   ; #:fourcc
   ; #:create-video-writer
   ; #:release-video-writer
   ; #:write-frame
))