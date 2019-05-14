#ifndef ForceGen_H
#define ForceGen_H
#include "RigidBody.h"
#include <list>

/**
* A force generator can be asked to add a force to one or more
* rigidbodyies.
*/
class ForceGenerator{
public:
	/**
	* Overload this in implementations of the interface to calculate
	* and update the force applied to the given rigidBody.
	*/
	virtual void updateForce(RigidBody *rigidBody ,float duration) = 0;
};

/**
* Holds all the force generators and the rigidBodies they apply to.
*/
class ForceRegistry {

protected:

	/**
	* Keeps track of one force generator and the rigidBody it
	* applies to.
	*/
	struct ForceRegistration{
		RigidBody* body;
		ForceGenerator *fg;
		
		bool operator == (const ForceRegistration other) const {
			return other.body == body && other.fg == fg;
		}
		
		ForceRegistration(RigidBody* rigidBody, ForceGenerator* fg) {
			this->body = rigidBody;
			this->fg = fg;
		}
	};

	/**
	* Holds the list of registrations.
	*/

	//typedef std::list<rigidBodyForceRegistration> Registry;

	std::list<ForceRegistration> registrations;

public:

	/**
	* Registers the given force generator to apply to the
	* given rigidBody.
	*/
	void add(RigidBody* rigidBody, ForceGenerator *fg);

	/**
	* Removes the given registered pair from the registry.
	* If the pair is not registered, this method will have
	* no effect.
	*/
	void remove(RigidBody* rigidBody, ForceGenerator *fg);

	/**
	* Clears all registrations from the registry. This will
	* not delete the rigidbodyies or the force generators
	* themselves, just the records of their connection.
	*/
	void clear();

	/**
	* Calls all the force generators to update the forces of
	* their corresponding rigidbodyies.
	*/
	void updateForces(float duration);
};

/**
* A force generator that applies a gravitational force. One instance
* can be used for multiple rigidBodies.
*/
class GravityForce : public ForceGenerator{

public:

	/** Creates the generator with the given acceleration. */
	GravityForce(const glm::vec3 gravity) :gravity(gravity) {};

	/** Applies the gravitational force to the given rigidBody. */
	virtual void updateForce(RigidBody *rigidBody ,float duration);

	inline const glm::vec3& getGravityAcc() const { return gravity; }

	inline void setGravityAcc(const glm::vec3& gravity){
		this->gravity = gravity;
	}

private:

	/** Holds the acceleration due to gravity. */
	glm::vec3 gravity;
};

/**
* A force generator that applies a drag force. One instance
* can be used for multiple rigidBodies.
*/
class DragForce : public ForceGenerator{

public:

	/** Creates the generator with the given coefficients. */
	DragForce(float k1, float k2) :k1(k1), k2(k2) {};

	/** Applies the drag force to the given rigidBody. */
	virtual void updateForce(RigidBody *rigidBody ,float duration);

private:

	/** Holds the velocity drag coefficient. */
	float k1;

	/** Holds the velocity squared drag coefficient. */
	float k2;

};

/**
* A force generator that applies a spring force.
*/
class Spring : public ForceGenerator{

	/** The rigidBody at the other end of the spring. */
	RigidBody *other;

	glm::vec3* connectionPoint;

	glm::vec3* otherConnectionPoint;

	/** Holds the spring constant. */
	float springConstant;
	
	/** Holds the rest length of the spring. */
	float restLength;

public:
	
	/** Creates a new spring with the given parameters. */
	Spring(RigidBody *other, glm::vec3* connectionPoint ,glm::vec3* otherConnectionPoint,
		float springConstant, float restLength) : springConstant(springConstant), restLength(restLength) {
		this->other = other;
		this->connectionPoint = connectionPoint;
		this->otherConnectionPoint = otherConnectionPoint;
	};
	
	/** Applies the spring force to the given rigidBody. */
	virtual void updateForce(RigidBody *rigidBody, float duration);
};

/**
* A force generator that applies a spring force, where
* one end is attached to a fixed point in space.
*/
class AnchoredSpring : public ForceGenerator{
public:
	
	/** Creates a new spring with the given parameters. */
	AnchoredSpring(glm::vec3 *anchor ,glm::vec3* connectionPoint,
		float springConstant, float restLength) : springConstant(springConstant), restLength(restLength){
		this->anchor = anchor;
		this->connectionPoint = connectionPoint;
	}
	
	/** Applies the spring force to the given rigidBody. */
	virtual void updateForce(RigidBody *rigidBody ,float duration);
private:
	
	/** The location of the anchored end of the spring. */
	glm::vec3* anchor;

	glm::vec3* connectionPoint;
	
	/** Holds the spring constant. */
	float springConstant;
	
	/** Holds the rest length of the spring. */
	float restLength;
};

/**
* A force generator that applies a spring force, where
* one end is attached to a fixed point in space.
*/
class FakeSpring : public ForceGenerator {
public:

	/** Creates a new spring with the given parameters. */
	FakeSpring(glm::vec3 *anchor, const glm::vec3& connectionPoint, glm::vec3* initialPosition,
		float springConstant ,float damping) : springConstant(springConstant) ,damping(damping) ,connectionPoint(connectionPoint){
		this->anchor = anchor;
		this->initialPosition = initialPosition;
	}

	/** Applies the spring force to the given rigidBody. */
	virtual void updateForce(RigidBody *rigidBody, float duration);
private:

	/** The location of the anchored end of the spring. */
	glm::vec3* anchor;

	glm::vec3* initialPosition;

	glm::vec3 connectionPoint;

	/** Holds the spring constant. */
	float springConstant;

	/** Holds the damping on the oscillation of the spring. */
	float damping;
};

#endif // !ForceGen_H
