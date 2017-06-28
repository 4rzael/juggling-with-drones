.PHONY: doc clean_doc trajectory_manager_doc swarm_manager_doc controller_doc

doc: clean_doc trajectory_manager_doc swarm_manager_doc controller_doc

clean_doc:
	rm -r doc

trajectory_manager_doc:
	mkdir -p doc/trajectory_manager
	cd crazyflie_trajectory_manager/src && pydoc -w *.py
	mv -f crazyflie_trajectory_manager/src/*.html doc/trajectory_manager/ || true

swarm_manager_doc:
	mkdir -p doc/swarm_manager
	cd juggling_swarm_manager/src && pydoc -w *.py
	mv -f juggling_swarm_manager/src/*.html doc/trajectory_manager/ || true

controller_doc:
	mkdir -p doc/controller
	cd juggling_controller/src && pydoc -w *.py
	mv -f juggling_controller/src/*.html doc/trajectory_manager/ || true
