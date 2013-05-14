﻿SAMPLE_ADC_CCP_P18F252
======================
Пример использования модулей АЦП и CCP. Модуль CCP работает в ШИМ режиме. К выводам МК, которые настроены как аналоговый вход АЦП, подключены три переменных резистора. 
В тот момент, когда уровень напряжения на входе AN0 будет больше либо равен уровню напряжения на входе AN1, произойдет включение модуляции ШИМ на выводе CCP1.
Длительность импульса ШИМ регулируется с помощью переменного резистора, подключенного ко входу AN3.
Программа создана в среде разработки MPLAB 8.8, для компиляции требуется  стандартная библиотека P18F252.INC. К примеру, прилагается схема для симулятора Proteus 7.8 и скомпилированный хекс-файл. Микроконтроллер работает на частоте 4Mhz, при питании 5 В. 
