# Пустой проект для stm32f407vg discovery

Идея этого пустого проекта заключается в том, что вы не зависите от среды разработки, вам нужна только Unix
подобная система. **Важный комменатарий, код в main.cpp пример, взятый откуда-то, чтобы проверить работу платы.**

![stm32f407vg discovery](image/stm32f407vg.jpeg)

# Установка

## Установка в mac OS

1. Install Xcode Command Line Tools (CLT). This will install *Make* and other UNIX goodies:
```
$ xcode-select --install
```
After the *Command Line Tools* were successfully installed, the remaining toolchain requirements can be installed using *Homebrew*.

2. Install *Homebrew*. Follow instructions available on [brew.sh][Homebrew]
3. Install GCC ARM Embedded Toolchain:
```
$ brew install homebrew/cask/gcc-arm-embedded
$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (Arm GNU Toolchain 12.2 (Build arm-12-mpacbti.34)) 12.2.1 20230214
Copyright (C) 2022 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

4. Install OpenOCD:
```
$ brew install openocd
$ openOCD --version
Open On-Chip Debugger 0.12.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
```

5. Install open source [stlink][texane/stlink]:
```
$ brew install stlink
$ st-info --version
v1.7.0
```

**P. s.** В этой [статье][altenativInstall] расписана альтернативная установка + как устанавливать на linux
 
## Установка в Linux

+- так же, как на mac OS

# Компиляция

Весь проект можно скомпилировать с помощью 'make all'

Добавлять новые '.с' файлы в проект в директорию 'src/' и прописывать в 'subdir.mk' 

'make clear' очистит все '.hex', '.map', '.elf' файлы.

С файлами '.o' и '.d' вам надо что то делать.

# Залитие на stm32f407vg discovery

```
$ sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \
"init; reset halt; flash write_image erase main.hex; "\  
"reset; exit"
```

Про отладку и д. т. читать в других статьях, таких как [эта][altenativInstall] или [можно в этой][oldsiet].


[oldsiet]:https://eax.me/openocd/
[altenativInstall]:https://cxemotexnika.org/2020/09/programmirovanie-stm32h747i-disco-iz-komandnoj-stroki-linux-macos/
[GNU ARM Embedded Toolchain]:https://developer.arm.com/open-source/gnu-toolchain/gnu-rm
[OpenOCD]:http://openocd.org/
[texane/stlink]:https://github.com/texane/stlink
[Homebrew]:https://brew.sh/
