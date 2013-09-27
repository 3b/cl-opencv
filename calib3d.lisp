;;;; -*- mode: lisp; indent-tabs-mode: nil -*-
;;;; calib3d.lisp
;;;; OpenCV bindings for SBCL
;;;; Camera calibration and 3D reconstruction
(in-package :cl-opencv)

(cffi:defcfun ("cvRedirectError" %redirect-error) :pointer
  (callback :pointer)
  (userdata :pointer)
  (old-userdata :pointer))

(cffi:defcallback error-callback :int ((status :int)
                                       (func-name :string)
                                       (error-message :string)
                                       (file-name :string)
                                       (line :int)
                                       (userdata :pointer))
  (declare (ignore userdata))
  (error "libcv error: ~s ~s in ~s (~s:~s)"
         status error-message func-name file-name line))

(%redirect-error (cffi:callback error-callback)
                 (cffi:null-pointer)
                 (cffi:null-pointer))

(cffi:defcstruct %cv-matrix
  (type :int)
  (step :int)
  (refcount :pointer)
  (hdr-refcount :int)
  (data :pointer)
  (rows :int)
  (cols :int))

(cffi:defcenum (cv-data-type :int)
  (:8u 0)
  (:8s 1)
  (:16u 2)
  (:16s 3)
  (:32s 4)
  (:32f 5)
  (:64f 6))

(defparameter *cv-lisp-data-types*
  ;; map of lisp types (and cv type enums) to cv type values and cffi types
  (make-hash-table :test 'equal))

;; lisp types that can go in a cv-matrix
(progn
  (setf (gethash '(unsigned-byte 8) *cv-lisp-data-types*) '(0 :uint8))
  (setf (gethash '(signed-byte 8) *cv-lisp-data-types*) '(1 :int8))
  (setf (gethash '(unsigned-byte 16) *cv-lisp-data-types*) '(2 :uint16))
  (setf (gethash '(signed-byte 16) *cv-lisp-data-types*) '(3 :int16))
  (setf (gethash '(signed-byte 32) *cv-lisp-data-types*) '(4 :int32))
  (setf (gethash 'single-float *cv-lisp-data-types*) '(5 :float))
  (setf (gethash 'double-float *cv-lisp-data-types*) '(6 :double))
  ;; might as well add the values from the enum too...
  (setf (gethash :8u *cv-lisp-data-types*) '(0 :uint8))
  (setf (gethash :8s *cv-lisp-data-types*) '(1 :int8))
  (setf (gethash :16u *cv-lisp-data-types*) '(2 :uint16))
  (setf (gethash :16s *cv-lisp-data-types*) '(3 :int16))
  (setf (gethash :32s *cv-lisp-data-types*) '(4 :int32))
  (setf (gethash :32f *cv-lisp-data-types*) '(5 :float))
  (setf (gethash :64f *cv-lisp-data-types*) '(6 :double)))

;; not going to type out the variants for now, use %make-matrix-type
;; to calculate them
(cffi:defctype cv-matrix-type :int)

(defconstant +cv-cn-shift+ 3)
(defconstant +cv-cn-max+ 512)
(defconstant +cv-cn-bits+ (integer-length +cv-cn-max+))

(defun %make-matrix-type (type count)
  "type should either be a cv-data-type enum or a lisp type"
  ;; fixme: add a compiler-macro to calculate this at compile time
  ;; when possible
  (let ((cv-matrix-type (if (numberp type)
                            type
                            (car (gethash type *cv-lisp-data-types*)))))
    (check-type cv-matrix-type (unsigned-byte 3))
    (dpb cv-matrix-type
         (byte +cv-cn-shift+ 0)
         (ash (1- count) +cv-cn-shift+))))

(defparameter *cv-types-to-lisp* #(((unsigned-byte 8) :uint8)
                                   ((signed-byte 8) :int8)
                                   ((unsigned-byte 16) :uint16)
                                   ((signed-byte 16) :int16)
                                   ((signed-byte 32) :int32)
                                   (single-float :float)
                                   (double-float :double)))

(defun %expand-matrix-type (type)
  "return list of number of channels, lisp type, and cffi type for cv
matrix type TYPE (an unsigned-byte)"
  (list* (1+ (ldb (byte +cv-cn-bits+ +cv-cn-shift+) type))
        (aref *cv-types-to-lisp* (ldb (byte +cv-cn-shift+ 0) type))))


(cffi:defcfun ("cvCreateMat" %create-matrix) cv-matrix
  (rows :int)
  (cols :int)
  (type cv-matrix-type))

(defun create-matrix (rows columns
                      &key (element-type '(unsigned-byte 8)) (channels 1))
  (%create-matrix rows columns
               (%make-matrix-type element-type channels)))

(cffi:defcfun ("cvCreateMatHeader" %create-matrix-header) cv-matrix
  (rows :int)
  (cols :int)
  (type cv-matrix-type))

(defun create-matrix-header (rows columns
                             &key (element-type '(unsigned-byte 8)) (channels 1))
  (%create-matrix-header rows columns
                         (%make-matrix-type element-type channels)))


(defun to-matrix (array &key (element-type (array-element-type array)))
  "convert a 2d or 3d array to a cv-matrix. Element type should be one
of (unsigned-byte 8), (signed-byte 16), (signed-byte 32),
single-float, or double-float. 2D arrays will be converted to
cv-matrix with 1 channel, 3d arrays should have 3rd dimension <=4,
which will be used as # of channels.
Result should be freed with release-matrix."
  (let* ((dims (array-dimensions array))
         (type (gethash element-type *cv-lisp-data-types*))
         (mat (%create-matrix (or (second dims) 1)
                              (first dims)
                              (%make-matrix-type (car type)
                                                 (or (third dims) 1))))
         (data (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'data)))
    #++(format t " dims=~s, type=~s (~s) mat=~s, data=~s~%  ~s~%"
            dims type element-type mat data
            (list (or (second dims) 1)
                  (first dims)
                  (%make-matrix-type (car type) (or (third dims) 1))))
    (macrolet ((copy (cffi-type)
                 `(loop for i below (array-total-size array)
                        do (setf (cffi:mem-aref data ,cffi-type i)
                                 (row-major-aref array i)))))
      (case (second type)
        (:uint8 (copy :uint8))
        (:int8 (copy :int8))
        (:uint16 (copy :uint16))
        (:int16 (copy :int16))
        (:int32 (copy :int32))
        (:float (copy :float))
        (:double (copy :double))
        (t (copy (second type)))))
    mat))

(defun from-matrix (mat &key 1d)
  "return contents of MAT as a 2d (3d of MAT has multiple channels) array"
  ;; possibly should add option to release it too?
  (destructuring-bind (channels lisp-type cffi-type)
      (%expand-matrix-type
       (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'type))
      (let* ((rows (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'rows))
             (cols (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'cols))
             (data (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'data))
             ;; fixme: use stride rather than copying directly
             #++
             (stride (cffi:foreign-slot-value mat '(:struct %cv-matrix) 'step))
             (array (make-array (cond
                                  (1d
                                   (* rows cols channels))
                                  ((> channels 1)
                                   (list cols rows channels))
                                  (t (list cols rows)))
                                :element-type lisp-type)))
        (macrolet ((copy (type)
                     `(loop for i below (array-total-size array)
                            do (setf (row-major-aref array i)
                                     (cffi:mem-aref data ,type i)))))
          ;; expand the copy loop out for expected types, so cffi can
          ;; optimize the accesses better
          (case cffi-type
            (:uint8 (copy :uint8))
            (:int8 (copy :int8))
            (:uint16 (copy :uint16))
            (:int16 (copy :int16))
            (:int32 (copy :int32))
            (:float (copy :float))
            (:double (copy :double))
            (t (copy cffi-type))))
        array)))

(cffi:defcfun ("cvReleaseMat" %release-matrix) :void
  (mat cv-matrix))

(defun release-matrix (mat)
  (cffi:with-foreign-object (p :pointer)
    (setf (cffi:mem-aref p :pointer) mat)
    #++(format t "releasing ~s ( @ ~s)~%" mat p)
    (%release-matrix p)
    #++(format t "done = ~s -> ~s~%" p (cffi:mem-aref p :pointer))))

;; todo: :initial-element arg?
(defmacro with-cv-matrix ((mat &key source rows columns
                                 (element-type nil element-type-p)
                                 (channels nil channels-p))
                          &body body)
  ;; should this return result of BODY, or return contents of MAT?
  (unless (or source (and rows columns))
    (error "must provide either source array or dimensions for with-cv-matrix"))
  (let ((%mat (gensym "MATRIX")))
    `(let ((,%mat
             ,(if source
                  `(to-matrix ,source ,@(when element-type-p
                                          `(:element-type ,element-type)))
                  `(create-matrix ,rows ,columns
                                  ,@(when element-type-p `(:element-type ,element-type))
                                  ,@(when channels-p `(:channels ,channels))))))
       (unwind-protect
            (let ((,mat ,%mat))
              ,@body)
         (when ,%mat
           #++(format t "with-cv-matrix release ~s~%" ,%mat)
           (release-matrix ,%mat)
           #++(format t "with-cv-matrix release done ~%"))))))

(defmacro with-cv-matrices ((&whole matrices
                               (mat &key source rows columns
                                      element-type
                                      channels)
                             &rest more-mats)
                            &body body)
  ;; todo: might be better to expand this all at once so there is only 1 uw-p?
  (declare (ignore mat source rows columns element-type channels more-mats))
  `(with-cv-matrix (,@(car matrices))
     ,@(if (cdr matrices)
          `((with-cv-matrices (,@(cdr matrices))
              ,@body))
          body)))

(cffi:defcfun ("cvReshape" %reshape) cv-matrix
  (arr cv-array)
  (header cv-matrix)
  (new-channels :int)
  (new-rows :int))

(defun reshape (cv-array &key channels rows)
  ";; returns new cv-matrix pointing to same data as CV-ARRAY, with
  ;; ROWS rows and CHANNELS channels (NIL or 0 means keep old value)
  ;; free with release-matrix"
  ;; fixme: figure out how to allocate an empty matrix properly
  (let ((mat (create-matrix-header 1 1)))
    (%reshape cv-array mat (or channels 0) (or rows 0))
    mat))


(cffi:defcstruct cv-scalar
  (values :double :count 4))

(cffi:defcfun ("cvSet1D" %set-1d) :void
  (arr cv-array)
  (index-0 :int)
  (value (:struct cv-scalar)))

(cffi:defcfun ("cvSet2D" %set-2d) :void
  (arr cv-array)
  (index-0 :int)
  (index-1 :int)
  (value (:struct cv-scalar)))

(cffi:defcfun ("cvSet3D" %set-3d) :void
  (arr cv-array)
  (index-0 :int)
  (index-1 :int)
  (index-2 :int)
  (value (:struct cv-scalar)))


(cffi:defcfun ("cvGet1D" %get-1d) (:struct cv-scalar)
  (arr cv-array)
  (index-0 :int))

(cffi:defcfun ("cvGet2D" %get-2d) (:struct cv-scalar)
  (arr cv-array)
  (index-0 :int)
  (index-1 :int))

(cffi:defcfun ("cvGet3D" %get-3d) (:struct cv-scalar)
  (arr cv-array)
  (index-0 :int)
  (index-1 :int)
  (index-2 :int))

(defmethod cffi:translate-into-foreign-memory (value (type cv-scalar-tclass) p)
  (cond
    ((numberp value)
       (setf (cffi:mem-aref p :double 0) value
             (cffi:mem-aref p :double 1) 0d0
             (cffi:mem-aref p :double 2) 0d0
             (cffi:mem-aref p :double 3) 0d0))
    ((listp value)
     (setf (cffi:mem-aref p :double 0) (float (pop value) 1d0)
           (cffi:mem-aref p :double 1) (float (or (pop value) 0d0) 1d0)
           (cffi:mem-aref p :double 2) (float (or (pop value) 0d0) 1d0)
           (cffi:mem-aref p :double 3) (float (or (pop value) 0d0) 1d0)))
    ((typep value '(vector double-float))
     (loop with l = (length value)
           for i below 4
           when (< i l)
             do (setf (cffi:mem-aref p :double i) (aref value i))
           else
             do (setf (cffi:mem-aref p :double i) 0d0)))
    ((vectorp value)
     (loop with l = (length value)
           for i below 4
           when (< i l)
             do (setf (cffi:mem-aref p :double i) (float (aref value i) 1d0))
           else
             do (setf (cffi:mem-aref p :double i) 0d0)))
    (t
     (error "can't translate ~s to cv-scalar?" value))))


(defun mref (cv-array &rest subscripts)
  "get an element from a CV-ARRAY (returns 4 double-float values,
regardless of number of channels in cv-array)"
  (let* ((v (ecase (length subscripts)
              (1 (apply #'%get-1d cv-array subscripts))
              (2 (apply #'%get-2d cv-array subscripts))
              (3 (apply #'%get-3d cv-array subscripts))))
         (p (getf v 'values)))
    (values (cffi:mem-aref p :double 0) (cffi:mem-aref p :double 1)
            (cffi:mem-aref p :double 2) (cffi:mem-aref p :double 3))))

(defsetf mref (cv-array &rest subscripts) (a b c d)
  `(progn
     ,(ecase (length subscripts)
        (1 `(%set-1d ,cv-array ,@subscripts (list ,a ,b ,c ,d)))
        (2 `(%set-2d ,cv-array ,@subscripts (list ,a ,b ,c ,d)))
        (3 `(%set-3d ,cv-array ,@subscripts (list ,a ,b ,c ,d))))
     ,a))

#++
(with-cv-matrix (m :source #2A((1 2) (3 4) (5 6)) :element-type '(signed-byte 16))
  (format t "created mat? ~s~%" m)
  (print (from-matrix m))
  (format t "copied mat?~%")
  (let ((m2 (reshape m :rows 1)))
    (format t "reshaped = ~s~%" m2)
    (format t "~s~%" (from-matrix m2 :1d t))
    (format t "m2[1] = ~s~%" (multiple-value-list (mref m2 1)))
    (%set-2d m2 0 0 123d0)
    (setf (mref m2 0 5) 234)
    (release-matrix m2)
    (format t "released reshaped"))
  (print (from-matrix m)))

(cffi:defbitfield svd-flags
  (:modify-a 1)
  (:transpose-u 2)
  (:transpose-v 4))

(cffi:defcfun ("cvSVD" svd) :void
  (a cv-array)
  (w cv-array)
  (u cv-array)
  (v cv-array)
  (flags svd-flags))

(cffi:defcenum solve-flags
  (:lu 0)
  (:svd 1)
  (:svd-sym 2) ;; called 'eig' in c++ api?
  (:cholesky 3)
  (:qr 4)
  ;; flags are set of enums + 1 that can be combined with others,
  ;; so just expand them by hand for now
  (:normal-lu 16)
  (:normal-svd 17)
  (:normal-svd-symmetric 18)
  (:normal-cholesky 19)
  (:normal-qr 20))

(cffi:defcfun ("cvSolve" solve) :int
  (src1 cv-array)
  (src2 cv-array)
  (dest cv-array)
  (flags solve-flags))


(cffi:defcstruct cv-term-criteria
  (type :int)
  (max-iter :int)
  (epsilon :double))

(defmethod cffi:translate-into-foreign-memory (value
                                               (type cv-term-criteria-tclass)
                                               p)
  ;; fixme: use a bitfield for type...
  (setf (cffi:foreign-slot-value p '(:struct cv-term-criteria) 'type)
        (logior (if (getf value :max-iter) 1 0) (if (getf value :epsilon) 2 0)))
  (setf (cffi:foreign-slot-value p '(:struct cv-term-criteria) 'max-iter)
        (or (getf value :max-iter) 0))
  (setf (cffi:foreign-slot-value p '(:struct cv-term-criteria) 'epsilon)
        (or (getf value :epsilon) 0d0)))


(cffi:defcfun ("cvCalibrateCamera" %calibrate-camera) :double
  (object-points cv-matrix)
  (image-points cv-matrix)
  (point-counts cv-matrix)
  (image-size :int64)
  (camera-matrix cv-matrix)
  (distortion-coefficients cv-matrix)
  (rotation-vectors cv-matrix)
  (translation-vectors cv-matrix)
  (flags :int)
  (term-crit (:struct cv-term-criteria)))

#++
(cffi:defcfun ("cvComputeCorrespondEpilines" %compute-correspond-epilines) :void
  ...)

#++
(cffi:defcfun ("cvConvertPointsHomogeneous" convert-points-homogeneous) :void
  ...)

#++
(cffi:defcfun ("cvCorrectMatches" correct-matches) :void
  ...)


#++
(cffi:defcfun ("cvDecomposeProjectionMatrix" decompose-projection-matrix) :void
  ...)

#++
(cffi:defcfun ("cvDrawChessboardCorners" draw-chessboard-corners) :void
  ...)

#++
(cffi:defcfun ("cvFindChessboardCorners" find-chessboard-corners) :void
  ...)


(cffi:defcfun ("cvFindExtrinsicCameraParams2" %solve-pnp) :void
  (object-points cv-matrix)
  (image-points cv-matrix)
  (camera-matrix cv-matrix)
  (distortion-coeffs cv-matrix)
  (rotation-vector cv-matrix)
  (translation-vectors cv-matrix)
  (use-extrinsic-guess :int)) ;; = 0


#++
(with-cv-matrix (object :source #2a((0.0 0.0 0.0)
                                    (0.1 0.0 0.0)
                                    (-0.1 0.0 0.0)
                                    (0.0 0.1 0.0))
                 :element-type 'single-float)
  (with-cv-matrix (image
                   :source
                   (make-array '(4 2)
                               :element-type 'double-float
                               :initial-contents
                               (loop for (i j) across #((337 244)
                                                        (456 245)
                                                        (220 242)
                                                        (337 124))
                                     collect (list (/ (- i 512) 512d0)
                                                   (/ (- j 384) 512d0)))))
    (with-cv-matrix (camera :source #2a ((1 0 0) (0 1 0) (0 0 1))
                     :element-type '(unsigned-byte 8))
      (with-cv-matrix (rvec :rows 1 :columns 3 :element-type 'double-float)
        (with-cv-matrix (tvec :rows 1 :columns 3 :element-type 'double-float)
          (%solve-pnp object image camera (cffi:null-pointer)
                      rvec tvec 0)
          (format t "rvec = ~s~%tvec = ~s (~s)~%"
                  (from-matrix rvec :1d t)
                  (from-matrix tvec :1d t)
                  (sqrt (reduce '+ (map 'list (lambda (x) (expt x 2))
                                        (from-matrix tvec :1d t))))))))))

(cffi:defcenum fm-method
  ;; not sure if these are intended as bitfields or enums?
  (:7points 1)
  (:8points 2)
  (:lmeds 4)
  (:ransac 8))

(cffi:defcfun ("cvFindFundamentalMat" %find-fundamental-matrix) :int
  (points1 cv-matrix)
  (points2 cv-matrix)
  (fundamental-matrix cv-matrix)
  (method fm-method)
  (param1 :double)
  (param2 :double)
  (status cv-matrix))

(cffi:defcfun ("cvFindHomography" %find-homography) :int
  (source-points cv-matrix)
  (dest-points cv-matrix)
  (homography cv-matrix)
  (method :int)
  (ransac-reproj-threshold :double)
  (mask cv-matrix))


(cffi:defcfun ("cvRodrigues2" %rodrigues2) :int
  (src cv-matrix)
  (dest cv-matrix)
  (jacobian cv-matrix))

#++
(cffi:defcfun ("cvStereoCalibrate" %stereo-calibrate) :double
  (object-points cv-matrix)
  (image-points1 cv-matrix)
  (image-points2 cv-matrix)
  (npoints cv-matrix)
  (camera-matrix1 cv-matrix)
  (dist-coefficients1 cv-matrix)
  (camera-matrix2 cv-matrix)
  (dist-coefficients2 cv-matrix)
  (image-size :int64)
  (rotation cv-matrix)
  (translation cv-matrix)
  (essential cv-matrix)
  (fundamental cv-matrix)
  (term-crit cv-term-criteria)
  (flags :int))
#++
(cffi:defcfun ("cvStereoRectify" %stereo-rectify) :void
  (camera-matrix1 cv-matrix)
  (camera-matrix2 cv-matrix)
  (dist-coefficients1 cv-matrix)
  (dist-coefficients2 cv-matrix)
  (image-size :int64)
  (rotation cv-matrix)
  (translation cv-matrix)
  (rectification1 cv-matrix)
  (rectification2 cv-matrix)
  (projection1 cv-matrix)
  (projection2 cv-matrix)
  (q cv-matrix)
  (flags :int)
  (alpha :double)
  (new-image-size :int64)
  (valid-pix-roi1 cv-rect)
  (valid-pix-roi2 cv-rect))

(cffi:defcfun ("cvStereoRectifyUncalibrated" %stereo-rectify-uncalibrated) :int
  (points1 cv-matrix)
  (points2 cv-matrix)
  (fundamental cv-matrix)
  (image-size :int64)
  (homography1 cv-matrix)
  (homography2 cv-matrix)
  (threshold :double))

(cffi:defcfun ("cvTriangulatePoints" %triangulate-points) :void
  (projection-matrix1 cv-matrix)
  (projection-matrix2 cv-matrix)
  (proj-points1 cv-matrix)
  (proj-points2 cv-matrix)
  (points-4d cv-matrix))
