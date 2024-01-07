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
