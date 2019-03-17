#include <GLFW/glfw3.h>

int main(void){
	GLFWwindow* window;

	if (!glfwInit())
		return -1;

	window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
	if (!window){
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	while (!glfwWindowShouldClose(window)){
		glClear(GL_COLOR_BUFFER_BIT);

		glfwSwapBuffers(window);

		glBegin(GL_POINTS);
		glVertex2d(0, 0);
		glVertex2d(0, 1);
		glVertex2d(0, 2);
		glVertex2d(0, 3);
		glEnd();

		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}