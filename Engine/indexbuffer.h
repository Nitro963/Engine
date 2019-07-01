#ifndef INDEX_BUFFER_H
#define INDEX_BUFFER_H
#pragma once
namespace renderer {
	class indexbuffer {
	private:
		unsigned int id;
		unsigned int count;
	public:
		indexbuffer(const unsigned int* data, unsigned int count);
		~indexbuffer();
		void bind() const;
		void unbind() const;
		inline unsigned int getcount() { return count; }
	};
}
#endif