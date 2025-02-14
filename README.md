Задание:


![image](https://github.com/user-attachments/assets/a30e8fc7-fad3-4bc1-9b5a-1a9ab8d34892)




Визуализация решения:


![image](https://github.com/user-attachments/assets/0a12df95-691d-412b-9baa-a596643437bd)





Поскольку в ТЗ не указано используемая архитектура процессора, но предположительно для запуска и проверки будет использоваться ПК с архитектурой процессора x86 x64, то в методе multiplyMatrices класса RobotKinematics используется AVX для обеспечения высокой производительности алгоритма (Через макросы реализовано отключение AVX, но предпочтительнее использовать if constexptr (C++17)).


Для ARM (планшеты, устройства для управления роботом) обычно применяют NEON/SVE.


Так же можно использовать библиотеку Eigen3, в которой уже применяются необходимые оптимизации алгоритмов, но о том можно ли его использовать или нет - не указано.
