# Estructuras-de-aceleración

### Performance inicial

La imagen inicial del tutorial 8, sin modificaciones, entrega la siguiente gráfica de performance:

![imagen_inicial](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/inicial.PNG)


### Creación de más objetos

Para modificar la escena, modifiqué la cantidad de cajas que se podían crear en la escena. Esto se puede ver en el código modificacion.cpp. Especfíficamente, en la línea de código #276 hasta #467 se crean 23 cajas adicionales. Adicionalmente, en la línea de código #616 a la #621 se crea una variable `factor` que sirve para crear una cantidad determinada de cajas con los parámetros de la primitiva `box`. En este caso se crean 1000 cajas adicionales. En las líneas de código #622 -644, se crean los GIs para las 23 cajas adicionales creadas. Finalmente, se crea el grupo de geometría de todas estas cajas creadas en las líneas de código #654 - 659. 

### Performance con aceleración

Para que la escena se carge con la estructura de aceleración, se deja activa la línea de código #692 `geometrygroup->setAcceleration( context->createAcceleration("Trbvh") )`. Esto da los siguientes resultados de performance: 

![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/performance_con_aceleracion.PNG)
![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/con_aceleracion.PNG)

Como se puede observar, la cantidad de FPS para generar más de 1000 cajas con estructura de aceleración es de 47.33.

### Performance sin aceleración

Ahora, para cargar los objetos sin la estructura de aceleracion, se deja activa la línes de código #693 `geometrygroup->setAcceleration( context->createAcceleration("NoAccel") );`. Esto da los siguientes resultados de performance:

![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/performance_sin_aceleracion.PNG)
![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/sin_aceleracion.PNG)

Como se puede observar, la cantidad de FPS para generar más de 1000 cajas sin estructura de aceleración es de 1.00.
