# The Basics

This portion of the guide is geared towards students new to software
development. We will touch on subjects we deem crucial to following
the rest of this guide effectively. It is recommended that all students
first skim through this section, then decide whether it is useful for them
to read further in-depth.

## Using the Command Line

Working in the terminal is usually the most daunting task when starting out in
software development, however there really is nothing to be afraid of. The
first question that may come to mind is: _"Why do we use the terminal/command
line?"_ Most notably it is a matter of convenience. _"Well if it is a matter
of convenience, then why is it so hard to do simple things?"_ This is likely
the perspective of anyone new to the field who has trained themselves on
the limiting graphical user-interface provided by our computers. The essence of
software development is learning how to properly communicate with your
computer. As you try to have your computer perform more complicated tasks, you
will start to learn that using a graphical user-interface creates a language
barrier. That is why we as programmers must learn a language more native to
the computer so we can better express the tasks we wish to complete. Poetic
analogies aside, that is the entire reason why the command line is often used.
Don't believe me? Well what if I asked you to delete all files on your system
ending with: ".txt"? Maybe that's that not too bad, but what if I asked you to
compile a C program on a remote server? That might be a little hard to do with
just the graphical interface. In essence, the command line is a portal into the
inner-workings of the operating system. This allows you to have more freedom in
your actions and enables a convinient workflow for performing more technical
operations.

Enough talking about the "why", let's start working on the "how".

Each operating system has it's own shell, which is just a program that uses the
command line as an input. Different shells have
different commands for interacting with the operating system. The most notable
shell is known as Bash, which is used by both Linux and Mac. Windows on the
other hand has two shells: Powershell and Command Prompt. This guide will only
focus on commands for the Bash shell, as it is the industry standard for
software development. Fear not Windows' users, in the `WSL2` tutorial we
discuss how one can run a Bash shell within their operating system.

> If you are using Windows to follow this tutorial, we recommend you first
> checkout the `02_wsl2.md` section. This will show you how to setup a Bash
> environment on Windows so that you can then follow along on your own machine.

We have curated the following list below which detail the
essential commands of Bash. At some point, you will need
to perform an action beyond the capability of this list. When that time comes
be sure to use the internet to your advantage.

### Current Working Directory

When executing commands on Bash, the result of those actions are
impacted by where you execute them within your file system. The 'where' is
often described as your current working directory, the folder which you are
executing commands within. At any point in time, you can find your working
directory by running:

```bash
pwd
```

Following the execution of that command, the terminal will **P**rint the path of the **W**orking **D**irectory. Which is why it is apporpriately named: 'pwd'.

Shells often have default working directories when you open a new terminal. For Bash,
this is the home directory.

### Changing Directories

What if the files we want to manipulate aren't in our working directory?
In that case, we need to change our working directory. Which can simply be done with:

```bash
cd <directory>
```

For instance, running this in the terminal would **c**hange your **d**irectory to the
"MyCode" folder:

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
data important to the operating system. The directory referenced by "~" is your home folder, which if you remember from earlier is our default working directory.

Here are two examples performing the same action:

```bash
cd ~/MyCode/
```

```bash
cd /home/<user>/MyCode/
```

These could be called from anywhere within the file system and they will have
the same outcome because of the prepended symbols which make it an absolute path.

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

### Text Editing

A common practice for quick file edits is to use a text editor within your terminal.
Vim and Nano are both programs made to perform command-line text editing. However both
these programs contain their own learning curves and aren't critical enough to be
fully covered within this tutorial. When starting out, we recommend you use graphical text
editors such as VSCode which are covered later in the tutorial.

## Version Control

Now that we have a basic understanding of how to manipulate files from our
terminal, let's explore how we use these files in a software development
context. Often when working on a project you may have multiple other people
editing the same folders or files. The question naturally arises, how do we ensure
our changes don't get overwritten by other collaborators? This is why we use
what is known as a Version Control System (VCS). For a majority of developers,
the go-to VCS is git, a program developed by the creators of Linux. We will give
you a quick walkthrough on how git works, however it should be known that mastering
git takes a lot more time and practice.

### Repositories

A repository is simply a collection of files and folders used in your project.
When setting up a project with git, you must first create a repository that tracks
all changes associated with the marked files. There are two ways you can create
a repository:

1.  `git init` : This command will create a repository in the current
    directory.
2.  `git clone <url>` : This command creates a copy of a remote repository.
    A folder containing the repository will be downloaded to your current
    directory.

> A remote
> repository is just a repository stored on a remote server. The go-to choice for
> storing remote repositories is GitHub. If you haven't created an account on
> GitHub already, then we highly suggest you do so. It's an excellent way to
> share code and checkout cool projects.

### Commits

Commits are how changes are stored within git. You often want to group related changes
within the same commit. There are three steps to creating a commit:

1. Check Your Changes:

   The first step to storing our changes is finding out what changes have actually
   been made. To do this, simply run:

   ```bash
   git status
   ```

   The result of this command will show which changes have been made and if they are staged.

2. Stage Your Changes:

   Staging changes is just a way to decide which changes to include in a commit.
   This ensures no unrelated or unwanted changes are being stored. To stage a change,
   you can run:

   ```bash
   git add <path to file or folder that has been changed>
   ```

   The list displayed in Step 1 shows the exact paths that can serve as inputs to this
   command. Additionally if you want to stage all changes that have been made, you
   can run:

   ```bash
   git add .
   ```

   Be aware though, this is a sure-fire way to include inappropriate changes.

3. Commit Your Changes

   This will wrap all staged changes into a commit
   and store it in the repository. We will find out later on the significance of
   commits. The command to commit changes is intuitively:

   ```bash
   git commit -m "Message to describe commit"
   ```
