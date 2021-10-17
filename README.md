//////////////////////// 
// Modo de Operación: // 
//////////////////////// 
Inclinometro/Goniometro desarrollado para la materia Medidas Electronicas I, en la UTN FRBA. 
Hardware: STM32F103C8TX, MPU6050, OLED 128x64: SSD1306, EEPROM: 24LC64. 
Grupo 3 
Curso: R4053 
Año: 2021 

//////////////////////// 
// Modo de Operación: // 
//////////////////////// 
•MODO1: "Medición" 
Se encarga de mostrar la inclinacion del dispositivo segun el eje horizontal del dispositivo, y el que segun se ve en
el silk-screen corresponde al eje Y del modulo.
Este eje es capaz de medir desde -180° a +180° absolutos, aunque en las proximidades de esos extremos comienza a fallar,
por lo tanto se lo programó para que tenga un uso real de -90° a +90°, lo cual es mas que suficiente ya que mayores angulos
que esos no permitirian observar el display. Por otro lado, como la idea es poder realizar mediciones relativas a partir
de una posición inicial determinada por el usuario, necesito de disponer la posibilidad de medir correctamente mas alla
de -90° o de +90°.
Cuando estamos en este modo el display muestra el angulo actual presedido de un simbolo > o < segun para que lado se
encuentra actualmente respecto del cero.
Si pulsamos el boton MODE pasamos al siguiente modo del dispositivo.
Si pulsamos el boton ZERO realizamos el aviso que a partir de ahora el angulo actual sea considerado como el cero, y de
esta forma se pueden realizar mediciones relativas. Si lo volvemos a pulsar en otra posicion elegimos nuevamente un 
angulo para realizar las mediciones. En caso de querer regresar a mediciones absolutas, es necesario apagar y volver a 
encerder el dispositivo.
Si pulsamos el boton HOLD congelamos en pantalla el angulo medido, y no se vuelve a medir hasta que sea pulsado nuevamente
por el usuario, o se apague y encienda nuevamente el dispositivo.  

•MODO2: "Calibracion"
En este modo podemos ir guardando las mediciones alcanzadas en funcion de un patron. El modo nos va a solicitar que midamos
unos 19 angulos en total que corresponden a saltos de a 10° desde los -90° hasta los +90°. De esta forma el dispositivo 
puede saber cuanto debe corregir para informar un valor correcto.
Es calibracion es guardada en la eeprom del dispotivo y por lo tanto una vez realizada no es necesario volver a hacerlo
incluso al apagar y volver a encender el dispotivo. Sin embargo, como cualquier dispositivo utilizado para mediciones, es
necesario realizar una calibracion cada cierto tiempo transcurrido independientemente del uso ue se le de.
Si pulsamos el boton MODE pasamos al siguiente modo del dispositivo.
Si pulsamos el boton ZERO pasamos al siguiente angulo a medir
Si pulsamos el boton HOLD guardamos la medicion actual como el valor correspondiente al angulo solicitado, salvo cuando
cuando estamos en el item nro 20 donde nos muestra "RESET CAL" en donde podemos resetear la calibracion guardada en la
EEPROM con los valores iniciales de fabrica. 

•Contenido del dispositivo:
3 pulsadores: Estan pensados para funcionamiento mono-tecla, mono-usuario
-Izquierdo: Se llama MODE
-Central: Se llama ZERO/NEXT
-Derecho: Se llama HOLD/SAVE
1 tecla de encendido y apagado para la bateria
1 puerto jack DC para alimentacion externa con 5VDC
1 LED rojo en el frente justo arriba del display que avisa cada vez que se alcanza un angulo multiplo de 10



__________________________

Fernando Castro Canosa - 2021
