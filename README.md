# FRC 2024

## Code Structure
This project will use the FSMSystem framework, representing each subsystem as a finite state machine.

[![Javadoc](https://github.com/Tino-FRC-2473/FRC2024/actions/workflows/javadoc.yml/badge.svg)](https://github.com/Tino-FRC-2473/FRC2024/actions/workflows/javadoc.yml)

Javadoc for this repo is available at [https://tino-frc-2473.github.io/FRC2024/](https://tino-frc-2473.github.io/FRC2024/)

## Code Conventions
We will base our code style off the [Sun Java style guide](https://www.oracle.com/technetwork/java/codeconventions-150003.pdf).
 * Indentation: tabs
 * Braces: endline
 * Wrap lines at 80 characters

Some additional naming guidance:
 * Variable names: camelCase
 * Booleans should start with "is" or "has" (ex. hasMotor, isPositive)

## Commit messages
Everyone should read and follow the rules in "[How to write a Git Commit Message](https://chris.beams.io/posts/git-commit/)."

# Integration Plan
- Local branches are personal development branches for work in progress code.
	- Naming convention: `dev/week-#/firstname_lastname/description-as-needed`

- Test branches are branches for code ready to test
	- Only test branches may be loaded onto a robot or test rig. Test branch merges from personal development branches do not require formal pull requests but should be peer reviewed and must pass build and style checks.
	- Test branches shall be created weekly based on main for each subteam. If multiple subteams need to test together, additional test branches can be created.
	- Naming convention: `test/week-#/subsystem/description-as-needed`

- Main branch will be where all subsystems come together.
	- Main branch merges require formal pull requests (peer & mentor reviewed) and must pass build and style checks. These changes should already have been tested on a test branch.
	- Main branch will serve as the weekly baseline for new development.

## MAXSwerveModule
Code provided by rev swerve template: https://github.com/REVrobotics/MAXSwerve-Java-Template

Copyright (c) 2009-2021 FIRST and other WPILib contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of FIRST, WPILib, nor the names of other WPILib
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
