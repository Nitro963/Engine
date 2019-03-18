#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H
#include"glm\glm.hpp"

class physicsObject {
public:
	physicsObject(glm::vec3 position ,glm::vec3 velocity):m_position(position) ,m_velocity(velocity){}
	inline const glm::vec3& getPosition() const { return m_position;}
	inline const glm::vec3& getVelocity() const { return m_velocity; }
	void integrate(float delta);
private:
	glm::vec3 m_position;
	glm::vec3 m_velocity;
	//glm::vec3 m_acceleration;
	//float mass;

};
#endif // !PHYSICSOBJECT_H
