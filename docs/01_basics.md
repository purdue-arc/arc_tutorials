# The Basics
This portion of the guide is geared towards students new to software
development. We will touch on subjects we deem crucial to following
the rest of this guide effectively. It is recommended that all students
first skim through this section, then decide whether it is useful for them
to read further in-depth.

## Using the Command Line
Working in the terminal is usually the most daunting task when starting out in
software development, however there really is nothing to be afraid of. The
first question that may come to mind is: "Why do we use the terminal/command
line?". Most notably it is a matter of convenience. "Well if it is a matter
of convenience, then why is it so hard to do simple things?". This is likely
the perspective of anyone new to the field, who has trained themselves on
the limiting graphical user-interface provided by our computers. The essence of
software development is learning how to properly communicate with your
computer. As you try to have your computer perform more complicated tasks, you 
will start to learn that using a graphical user-interface is equivalent to
speaking in the computer's non-native language. That is why we as programmers
must learn the computer's language so we can better express the tasks we wish
to complete. Poetic analogies aside, that is the entire reason why the command
line is often used. Don't believe me? Well what if I asked you to delete all
files on your system ending with: ".txt"? Maybe that's that not too bad, but
what if I asked you to compile a C program on a remote server? That might be
a little hard to do with just the graphical interface. In essence, the command
line is just a portal into the inner-workings of the operating system. This
allows to have more freedom in your actions and enables a convinient workflow
for performing more technical actions.

Enough talking about the "why", let's start working on the "how". 

Each operating system has it's own shell, which is just a program that uses the
command line as an input. This means that certain operating systems may have
different commands for interacting with the shell. The most notable being the
Bash shell, which is used by both Linux and Mac. Windows users on the other
hand have two shells: Powershell and Command Prompt. Fear not however, in the
next tutorial we will discuss how Windows users can run the Bash shell and more
on their computer. Knowing this, the following guide will be focused directly
on commands available within Bash. This is far from being an inclusive list,
but it includes what we believe are important when starting your journey to
mastering the command line.

### Changing Directories
Shells often run commands at a default location within your filesystem. In Bash,
this default location is usually the Home directory. If we wanted to run
commands from another location however, then we need to use the following 
command:

```bash
cd <directory>
```

For instance, running this in the terminal would move you into the "MyCode" folder:

```bash
cd ./MyCode/
```
Sometimes you may want to reference a file location from a different context.
In the case above, we referenced our folder from the current folder by
using: ". /". Using the ". /" symbol tells the computer to perform
actions from the current directory. What if the "MyCode" folder was actually
in the folder above my current directory however? Then we would have to use
the ".. /" symbol, which indicates the parent directory to my current directory.
Here's what this looks like in practice:

```bash
cd ../MyCode/
```

This pattern continues for further parent folders, let's pretend "MyCode" was
3 directories above mine:

```bash
cd ../../../MyCode/
```

So far, all examples have been referencing folders relative to my current folder. You can also specify folders from an
absolute path. Bash has these two common absolute symbols:
"/" and "~". The directory referenced by "/" is your root folder and contains
data important to the operating system. The directory referenced by "~" is your home folder.

Here are both performing the same action:

```bash
cd ~/MyCode/
```
```bash
cd /home/<user>/MyCode/
```

These could be called from anywhere within the file system and they will have
the same outcome because of the prepended symbols.

### Listing Files/Folders
Now you are in the correct directory, but how do we see what files are there?
This is simply done by entering:

```bash
ls
```

Following the execution of this command, a list of folders and files will
be printed.

### File Manipulation
Copy files from one place to another:

```bash
cp <file to copy> <location to copy to>
```

You can also copy folders using the recursive flag:

```bash
cp -r <folder to copy> <location to copy to>
```

Flags are simply additional command arguments that modify how a command might
be executed.

Lastly you can move and rename folders and files with the move command:

```bash
mv <file / folder> <destination>
```

If you want to rename a file or folder, simply change it's name in the
destination argument. For example, lets say I want to rename "MyCode" to
"OurCode":

```bash
mv MyCode/ OurCode/
```

If an "OurCode" folder already existed, then it would only move the "MyCode" folder to the "OurCode" folder.

These concepts remain the same for files.

## Version Control
