#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H
#include"glm\glm.hpp"

class physicalObject {
public:
	physicalObject(const glm::vec3 position ,float mass):m_position(position) ,m_mass(mass) ,m_acceleration(0 ,0 ,0) ,m_velocity(0 ,0 ,0){}
	inline const glm::vec3& getPosition() const { return m_position;}
	inline const glm::vec3& getVelocity() const { return m_velocity; }
	inline const glm::vec3& getAcceleration() const { return m_acceleration; }
	inline const float getMass() const { return m_mass; }

	void integrate();
	void applyForce(const glm::vec3& force);
private:
	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;
	float m_mass;
};
#endif // !PHYSICSOBJECT_H
