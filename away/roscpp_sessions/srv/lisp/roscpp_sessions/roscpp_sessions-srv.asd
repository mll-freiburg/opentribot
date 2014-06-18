
(in-package :asdf)

(defsystem "roscpp_sessions-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "set_variable" :depends-on ("_package"))
    (:file "_package_set_variable" :depends-on ("_package"))
    (:file "get_variable" :depends-on ("_package"))
    (:file "_package_get_variable" :depends-on ("_package"))
    (:file "simple_session" :depends-on ("_package"))
    (:file "_package_simple_session" :depends-on ("_package"))
    (:file "add_variables" :depends-on ("_package"))
    (:file "_package_add_variables" :depends-on ("_package"))
    ))
