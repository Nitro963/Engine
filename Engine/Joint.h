#ifndef JOINT_H
#define JOINT_H
#include "RigidBody.h"
#include <list>

class Joint {
public:
	virtual void solveConstraint(RigidBody* body) = 0;
};
/**
* Holds all the force generators and the rigidBodies they apply to.
*/
class JointRegistry {

protected:

	/**
	* Keeps track of one force generator and the rigidBody it
	* applies to.
	*/
	struct JointRegistration {
		RigidBody* body;
		Joint *joint;

		bool operator == (const JointRegistration other) const {
			return other.body == body && other.joint == joint;
		}

		JointRegistration(RigidBody* rigidBody, Joint* joint) : body(rigidBody), joint(joint) {}
	};

	std::list<JointRegistration> registrations;
public:
	//Registers the given force generator to apply to the
	//given rigidBody.

	void add(RigidBody* rigidBody, Joint *joint);

	//Removes the given registered body from the registry if fun returned true
	void remove_if(bool(*fun)(RigidBody*&));

	//Removes the given registeration from the registry if fun returned true
	void remove_if(bool(*fun)(JointRegistration& R));

	void remove(RigidBody*&);

	//Clears all registrations from the registry. This will
	//not delete the rigidbodies or the joints
	//themselves, just the records of their connection.

	void clear();

	//Calls all the joints to solve for the constraints of
	//their corresponding rigidbodies.
	void solveConstraints(float duration);
};
class FixedJoint : public Joint {
public:
	FixedJoint(const glm::vec3& pos) : pos(pos) {}
	virtual void solveConstraint(RigidBody* body);
private:
	glm::vec3 pos;
};

#endif // !JOINT_H