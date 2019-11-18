# Estructuras-de-aceleración

### Performance inicial

La imagen inicial del tutorial 8, sin modificaciones, entrega la siguiente gráfica de performance:

![imagen_inicial](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/inicial.PNG)


### Creación de más objetos

Para modificar la escena, modifiqué la cantidad de cajas que se podían crear en la escena. Esto se puede ver en el código modificacion.cpp. Especfíficamente, en la línea de código #276 hasta #467 se crean 23 cajas adicionales. Adicionalmente, en la línea de código #616 a la #621 se crea una variable `factor` que sirve para crear una cantidad determinada de cajas con los parámetros de la primitiva `box`. En este caso se crean 1000 cajas adicionales. En las líneas de código #622 -644, se crean los GIs para las 23 cajas adicionales creadas. Finalmente, se crea el grupo de geometría de todas estas cajas creadas en las líneas de código #654 - 659. 

### Performance con aceleración


![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/performance_con_aceleracion.PNG)
![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/con_aceleracion.PNG)


### Performance sin aceleración

![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/performance_sin_aceleracion.PNG)
![](https://github.com/wilmerodriguez/Estructuras-de-aceleracion/blob/master/sin_aceleracion.PNG)
