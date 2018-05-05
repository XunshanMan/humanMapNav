
(cl:in-package :asdf)

(defsystem "doornumber-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "boardArray" :depends-on ("_package_boardArray"))
    (:file "_package_boardArray" :depends-on ("_package"))
    (:file "board" :depends-on ("_package_board"))
    (:file "_package_board" :depends-on ("_package"))
  ))