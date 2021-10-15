
(in-package :cl-meld)

(defparameter *file* nil "Compiled file path.")
(defparameter *ast* nil "Abstract Syntax Tree.")
(defparameter *code* nil "Virtual Machine instructions.")
(defparameter *code-rules* nil "Virtual Machine instructions for each rule.")
(defparameter *has-exists-p* nil "If any exists construct exists in the program.")

(define-symbol-macro *definitions* (definitions *ast*))
(define-symbol-macro *node-definitions* *definitions*)
(define-symbol-macro *clauses* (clauses *ast*))
(define-symbol-macro *node-var-axioms* (node-var-axioms *ast*))
(define-symbol-macro *thread-var-axioms* (thread-var-axioms *ast*))
(define-symbol-macro *node-const-axioms* (node-const-axioms *ast*))
(define-symbol-macro *thread-const-axioms* (thread-const-axioms *ast*))
(define-symbol-macro *nodes* (nodes *ast*))
(define-symbol-macro *externs* (externs *ast*))
(define-symbol-macro *functions* (functions *ast*))
(define-symbol-macro *directives* (directives *ast*))
(define-symbol-macro *consts* (consts *ast*))
(define-symbol-macro *processes* (processes *code*))
(define-symbol-macro *consts-code* (consts *code*))
(define-symbol-macro *function-code* (functions *code*))
(define-symbol-macro *exported-predicates* (exported-predicates *ast*))
(define-symbol-macro *imported-predicates* (imported-predicates *ast*))

(defun set-abstract-syntax-tree (ast) (setf *ast* ast))
