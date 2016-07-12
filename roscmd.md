catkin_create_pkg <package-name> <dependencies>
catkin_init_workspace
rospack depends1 <package-name>
rospack depends <package-name>
catkin_make --source <src-location>
catkin_make install --source <src-location>
rosrun <pkg> <exe> __name:=<remapped-node-name>
rosnode ping <node>
rostopic echo <topic>
rostopic pub <topic> <msg-type> -1 (-r 1) (--) '<args-in-YAML>' (-- means no options following, in order to take negative numbers)
rostopic list -v (verbose)
rostopic type <topic>|rosmsg show
rosservice type <service>|rossrv show

rospack
    depends
    depends1
    find
    list
    plugins
    profile
    rosdep
    rosdep0
    
    cflags-only-I
    cflags-only-other
    
    depends-indent
    depends-manifests
    depends-msgsrv
    depends-on
    depends-on1
    depends-why
    
    help
    
    langs
    
    libs-only-l
    libs-only-L
    libs-only-other
    
    list-duplicates
    list-names
    
    vcs
    vcs0
    
rosnode
	ping	test connectivity to node
	list	list active nodes
	info	print information about node
	machine	list nodes running on a particular machine or list machines
	kill	kill a running node
	cleanup	purge registration information of unreachable nodes

rostopic
    bw	    display bandwidth used by topic
	echo	print messages to screen
	find	find topics by type
	hz	    display publishing rate of topic
	info	print information about active topic
	list	list active topics
	pub	    publish data to topic
	type	print topic type

rosservice
	args	print service arguments
	call	call the service with the provided args
	find	find services by service type
	info	print information about service
	list	list active services
	type	print service type
	uri	    print service ROSRPC uri

rosparam
	set	    set parameter
	get	    get parameter
	load	load parameters from file
	dump	dump parameters to file
	delete	delete parameter
	list	list parameter names


