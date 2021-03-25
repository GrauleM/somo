======
SoMo Manipulators
======

SoMo provides an easy way to generate continuum manipulators. **Manipulators** can be comprised of several serially-chained **actuators** (:class:`~somo.sm_actuator_definition.SMActuatorDefinition`), made up of a series of **links** (:class:`~somo.sm_link_definition.SMLinkDefinition`) connected by spring-loaded **joints** (:class:`~somo.sm_joint_definition.SMJointDefinition`), 

To actually implement a manipulator, you can define it in a dictionary or yaml/json file, and load the definition as a :class:`~somo.sm_manipulator_definition.SMManipulatorDefinition` object. The lower-level definitions are taken care of internally.


.. autoclass:: somo.sm_link_definition.SMLinkDefinition
   :members:
   :undoc-members:


.. autoclass:: somo.sm_joint_definition.SMJointDefinition
   :members:
   :undoc-members:


.. autoclass:: somo.sm_actuator_definition.SMActuatorDefinition
   :members:
   :undoc-members:


.. autoclass:: somo.sm_manipulator_definition.SMManipulatorDefinition
   :members:
   :undoc-members:
