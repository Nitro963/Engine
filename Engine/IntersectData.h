#ifndef INTERSECTDATA
#define INTERSECTDATA

class intersectData {
public:
	inline intersectData(const bool doseIntersect, const float distance) :m_doseIntersect(doseIntersect), m_distance(distance) {}
	inline bool getDoseIntersect() const { return m_doseIntersect; }
	inline float getDistance()const { return m_distance; }
private:
	const float m_distance;
	bool m_doseIntersect;
};
#endif // !INTERSECTDATA
