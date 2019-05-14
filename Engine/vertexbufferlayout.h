#pragma once
#include<vector>
#include"basic.h"
namespace renderer {
	struct vertexbufferelement
	{
		unsigned int type;
		unsigned int count;
		unsigned char normalized;

		static unsigned int getsize(unsigned int type) {
			switch (type) {
			case GL_FLOAT: return 4;
			case GL_UNSIGNED_INT: return 4;
			case GL_UNSIGNED_BYTE: return 1;
			case GL_INT: return 4;
			}

			ASSERT(false);
			return 0;
		}

	};
	class vertexbufferlayout {
	private:
		std::vector<vertexbufferelement> m_element;
		unsigned m_stride;

	public:
		vertexbufferlayout() :m_stride(0) {};
		template<typename T>
		void push(unsigned int count) {
			static_assert(false);
		}

		template<>
		void push<float>(unsigned int count) {
			m_element.push_back({ GL_FLOAT ,count ,GL_FALSE });
			m_stride += count *  vertexbufferelement::getsize(GL_FLOAT);
		}

		template<>
		void push<unsigned int>(unsigned int count) {
			m_element.push_back({ GL_UNSIGNED_INT ,count ,GL_FALSE });
			m_stride += count * vertexbufferelement::getsize(GL_UNSIGNED_INT);
		}

		template<>
		void push<int>(unsigned int count) {
			m_element.push_back({ GL_INT ,count ,GL_FALSE });
			m_stride += count * vertexbufferelement::getsize(GL_INT);
		}

		template<>
		void push<unsigned char>(unsigned int count) {
			m_element.push_back({ GL_UNSIGNED_BYTE ,count ,GL_TRUE });
			m_stride += count * vertexbufferelement::getsize(GL_UNSIGNED_BYTE);
		}

		inline const std::vector<vertexbufferelement>& getelements() const { return m_element; }
		inline const int getstride()const { return m_stride; }
	};
}