#ifndef ForceGen_H
#define ForceGen_H
#include "RigidBody.h"
#include <list>

/**
* A force generator can be asked to add a force to one or more
* rigidbodies.
*/
class ForceGenerator {
public:
	/**
	* Overload this in implementations of the interface to calculate
	* and update the force applied to the given rigidBody.*/
	virtual void updateForce(RigidBody *rigidBody, float duration) = 0;
};
class TimedMotorJoint;
/**
* Holds all the force generators and the rigidBodies they apply to.
*/
class ForceRegistry {

protected:

	/**
	* Keeps track of one force generator and the rigidBody it
	* applies to.
	*/
	struct ForceRegistration {
		RigidBody* body;
		ForceGenerator *fg;

		bool operator == (const ForceRegistration other) const {
			return other.body == body && other.fg == fg;
		}

		ForceRegistration(RigidBody* rigidBody, ForceGenerator* fg) : body(rigidBody), fg(fg) {}
	};

	std::list<ForceRegistration> registrations;
public:
	//Registers the given force generator to apply to the
	//given rigidBody.

	void add(RigidBody* rigidBody, ForceGenerator *fg);

	//Removes the given registered body from the registry if fun returned true
	void remove_if(bool(*fun)(RigidBody*&));

	//Removes the given registeration from the registry if fun returned true
	void remove_if(bool(*fun)(ForceRegistration& R));

	void remove(RigidBody*& body);

	void remove(ForceGenerator* fg);

	//Clears all registrations from the registry. This will
	//not delete the rigidbodies or the force generators
	//themselves, just the records of their connection.

	void clear();

	//Calls all the force generators to update the forces of
	//their corresponding rigidbodies.	
	void updateForces(float duration);
};

/**
* A force generator that applies a gravitational force. One instance
* can be used for multiple rigidBodies.
*/
class GravityForce : public ForceGenerator {

public:
	//Creates the generator with the given acceleration.
	GravityForce(const glm::vec3 gravity) : gravity(gravity) {};
	GravityForce(const float& planetMass, const float& planetRadius) : gravity(glm::vec3(0, -G * planetMass / planetRadius, 0)) {}
	static const float G;
	inline static GravityForce* moonGravity() { return new GravityForce(glm::vec3(0, -1.62, 0)); }
	inline static GravityForce* EarthGravity() { return new GravityForce(glm::vec3(0, -9.807, 0)); }
	inline static GravityForce* saturnGravity() { return new GravityForce(glm::vec3(0, -10.44, 0)); }
	inline static GravityForce* jupiterGravity() { return new GravityForce(glm::vec3(0, -24.79, 0)); }

	//Applies the gravitational force to the given rigidBody.
	virtual void updateForce(RigidBody *rigidBody, float duration) override;

	inline const glm::vec3& getGravityAcc() const { return gravity; }

	inline void setGravityAcc(const glm::vec3& gravity) {
		this->gravity = gravity;
	}

private:

	//Holds the acceleration due to gravity.
	glm::vec3 gravity;
};

/**
* A force generator that applies a drag force. One instance
* can be used for multiple rigidBodies.
*/
class DragForce : public ForceGenerator {

public:

	//Creates the generator with the given coefficient.
	DragForce(float C) :C(C) {};

	//Applies the drag force to the given rigidBody.
	virtual void updateForce(RigidBody *rigidBody, float duration) override;

private:

	//Holds the velocity drag coefficient.
	float C;
};

class MotorJoint : public ForceGenerator {
public:
	MotorJoint(const glm::vec3& force, const glm::vec3& pt) : force(force), pt(pt) {}
	virtual void updateForce(RigidBody *RigidBody, float duration) override;
protected:
	//the point where to apply force in the rigid body local space
	glm::vec3 pt;
	//the force to be applied
	glm::vec3 force;
};

class TimedMotorJoint : public MotorJoint {
public:
	TimedMotorJoint(const float& t ,const glm::vec3& force, const glm::vec3 pt) : t(t) ,MotorJoint(force, pt){}
	virtual void updateForce(RigidBody *RigidBody, float duration) override;
private:
	float t;
};

#endif // !ForceGen_H