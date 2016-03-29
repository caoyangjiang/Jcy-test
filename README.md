How to compile:
==============

**For LINUX**

~~~~~~~~~~~~~~~~~~~~~
    mkdir build

    cd build

    cmake ..

    make
~~~~~~~~~~~~~~~~~~~~~

**For WINDOWS**

~~~~~~~~~~~~~~~~~~~~~
    mkdir build

    cd build

    cmake .. -G "Visual Studio 12 Win64"
~~~~~~~~~~~~~~~~~~~~~

- Need to specify the compiler otherwise it will be defaulted to the 32 bit compiler, this example is assuming compiling in visual studio 2013

- click on the .sln

- compile (F7) on visual studio

<br />

Setup coding style
==============

**For Linux: Using Sublime 3 as main IDE**

- Install prerequisites

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install python (Ubuntu)
    sudo yum     install python (CentOS)
~~~~~~~~~~~~~~~~~~~~~

- Open Sublime

- Ctrl + `

- copy command from https://packagecontrol.io/installation

- press enter and restart Sublime

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install python-pip (Ubuntu)
    sudo yum     install python-pip (CentOS)

    sudo pip install cpplint

    cd hvr-sample

    cp /usr/local/lib/python2.7/dist-packages/cpplint.py style-check/ (Ubuntu)
    cp /usr/lib/python2.7/site-packages/cpplint.py style-check/       (CentOS)
~~~~~~~~~~~~~~~~~~~~~

- (Copy cpplint.py to style-check when repo is cloned to local machine)

- Setup pre-push hook

~~~~~~~~~~~~~~~~~~~~~
    cp style-check/pre-push .git/hooks/pre-push
~~~~~~~~~~~~~~~~~~~~~

- Open Sublime

- ctrl + shift + p

- select "package control: install package"

- search and install SublimeLinter

- search and install SublimeLinter-cpplint

- add following to "Preferences"->"Package settings"->"SublimeLinter"->"Settings-User" under the cpplint section

~~~~~~~~~~~~~~~~~~~~~
 "cpplint": {
    "@disable": false,
    "args": [],
    "excludes": [],
    "filter": "-whitespace/braces,-whitespace/newline,-runtime/references,-build/c++11"
}
~~~~~~~~~~~~~~~~~~~~~

- If Settings-User is empty, "ctrl + shift + p" -> "SublimeLinter: enable" 

- build the latest llvm/clang by following http://clang.llvm.org/get_started.html (obtain latest release)  
- ctrl + shift + p

- select "package control: install package"

- search and install clang format

- "Preferences"->"Package settings"->"Clang Format"->"Settings-User"

- add in (To find "clang-format", use "updatedb -> locate clang-format")

~~~~~~~~~~~~~~~~~~~~~
{
    "binary": "/path/to/clang/bin/clang-format",

    "style": "Custom",

    "format_on_save": true
}
~~~~~~~~~~~~~~~~~~~~~

- add following style to "Preferences"->"Package settings"->"Clang Format"->"Custom Style-User" 

~~~~~~~~~~~~~~~~~~~~~
{
    "BasedOnStyle": "Google",
    "AlignConsecutiveAssignments": true,
    "AlignTrailingComments": true,
    "AllowAllParametersOfDeclarationOnNextLine": true,
    "AllowShortCaseLabelsOnASingleLine": true,
    "AllowShortIfStatementsOnASingleLine": true,
    "AllowShortFunctionsOnASingleLine": "None",
    "AllowShortLoopsOnASingleLine": true,
    "BinPackArguments": false,
    "BinPackParameters": false,
    "BreakBeforeBraces": "Allman",
    "BreakConstructorInitializersBeforeComma": true,
    "ColumnLimit": 80
}
~~~~~~~~~~~~~~~~~~~~~

- ctrl + shift + p

- select "package control: install package"

- search and install SublimeLinter-contrib-cmakelint

- add following to "Preferences"->"Package settings"->"SublimeLinter"->"Settings-User" under the cmakelint section

~~~~~~~~~~~~~~~~~~~~~
"cmakelint": {
    "@disable": false,
    "args": [
        "--filter=-convention/filename"
    ],
    "excludes": []
}
~~~~~~~~~~~~~~~~~~~~~

- ctrl + shift + p

- search and install EditorConfig to get basic document consistency


**For Windows: Using Visual Studio as main IDE**

- Download and Install the latest python, add path for all users

~~~~~~~~~~~~~~~~~~~~~
    pip install cpplint

    copy C:\Python27\Scripts\cpplint-script.py style-check\cpplint.py
~~~~~~~~~~~~~~~~~~~~~

- Setup pre-push hook

~~~~~~~~~~~~~~~~~~~~~
    copy "style-check\pre-push" ".git\hooks\pre-push"  
~~~~~~~~~~~~~~~~~~~~~

- in Visual Studio, Tools -> External Tools -> Add. Name it cpplint, adjust the filter according to CPPLINT.cfg (this is in the repo)

~~~~~~~~~~~~~~~~~~~~~
    Title             : cpplint
    Command           : C:\Python27\python.exe
    Arguments         : C:\Python27\Scripts\cpplint-script.py --filter= --extensions=h,hpp,c,cpp $(ItemPath)
    Initial Directory : $(ItemDir)
~~~~~~~~~~~~~~~~~~~~~

- Move the newly created cpplint to the very top

- Download and install the latest clang-format plugin from http://llvm.org/builds/

- Go to Tools->Options->LLVM/Clang->ClangFormat and put down "File" in the Style option field.

- This will search for a .clang-format file in where the source is it's parent directory

- Install Visual Commander in Tools -> Extensions and Updates

- VCMD -> Extensions -> Add. Name it cpplint_clangformat. Choose C# as a language. Copy/Past the following code. Now Save, Compile, Install.

~~~~~~~~~~~~~~~~~~~~~
    using EnvDTE;

    using EnvDTE80;

     

    public class E : VisualCommanderExt.IExtension

    {

        public void SetSite(EnvDTE80.DTE2 DTE_, Microsoft.VisualStudio.Shell.Package package)

        {

            DTE = DTE_;
        
            events = DTE.Events;
        
            documentEvents = events.DocumentEvents;
        
            documentEvents.DocumentSaved += OnDocumentSaved;  

        }

     

        public void Close()

        {

            documentEvents.DocumentSaved -= OnDocumentSaved;

        }

     

        private void OnDocumentSaved(EnvDTE.Document doc)

        {

            if(doc.Language == "C/C++")

            {

                DTE.ExecuteCommand("Edit.SelectAll");

                DTE.ExecuteCommand("Tools.ClangFormat");

                DTE.ExecuteCommand("Tools.ExternalCommand1");

                DTE.ExecuteCommand("File.SaveAll");

            }

        }

     

        private EnvDTE80.DTE2 DTE;

        private EnvDTE.Events events;

        private EnvDTE.DocumentEvents documentEvents;

    }
~~~~~~~~~~~~~~~~~~~~~

- This will invoke cpplint and clang format whenever a c++ related file is saved

- in Visual Studio, Tools -> Extensions and Updates -> , search for editorconfig and install it

- in Visual Studio, Tools -> Extensions and Updates -> , search for cmake tools for visual studio and install it

<br />

Doxygen
--------------

- To build with doxygen 

~~~~~~~~~~~~~~~~~~~~~
   cmake .. -DBUILD_DOCUMENTATION=ON 
~~~~~~~~~~~~~~~~~~~~~

**For Linux**

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install doxygen texlive-full graphviz
~~~~~~~~~~~~~~~~~~~~~

**For Windows**

- Download and install the latest doxygen 

- Download and install the latest MikTeX 

- Download and install the latest Graphviz 

- Manually add Graphviz to system PATH 

CUDA
--------------

**For Linux**

- Download the latest cuda deb file on NVIDIA's website 

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get remove --purge cuda*

    sudo dpkg -i cuda-repo-ubuntuXXXX_X.X-XX_amd64.deb

    sudo apt-get update

    sudo apt-get install cuda
~~~~~~~~~~~~~~~~~~~~~

- Add system path

~~~~~~~~~~~~~~~~~~~~~
    gedit ~/.bashrc
~~~~~~~~~~~~~~~~~~~~~

- export PATH=/usr/local/cuda/bin:$PATH

- export LD_LIBRARY_PATH=/usr/local/cuda/lib:$LD_LIBRARY_PATH

- export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

- Copy and compile samples

~~~~~~~~~~~~~~~~~~~~~
    cuda-install-samples-X.X.sh ~

    cd NVIDIA_CUDA-X.X_Samples

    make
~~~~~~~~~~~~~~~~~~~~~

- Test samples

~~~~~~~~~~~~~~~~~~~~~
    cd bin/x86_64/linux/release
    
    ./deviceQuery

    ./bandwidthTest
    
    ./smokeParticles
~~~~~~~~~~~~~~~~~~~~~

**For Windows**

- Download the latest cuda

- Follow instructions on express installation

OpenCV
--------------

**For Linux**

- Install prerequisites

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install cmake libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
~~~~~~~~~~~~~~~~~~~~~

- Get the latest OpenCV

~~~~~~~~~~~~~~~~~~~~~
    git clone https://github.com/Itseez/opencv

    cd opencv
    
    mkdir build
    
    cd build

    ccmake ..
~~~~~~~~~~~~~~~~~~~~~

- Add/Remove modules accordingly

~~~~~~~~~~~~~~~~~~~~~
    make

    sudo make install
~~~~~~~~~~~~~~~~~~~~~

**For Windows**

- Download the latest OpenCV

- Set "OpenCV_DIR" system variable into "XXX\opencv\build"

- Set system variable at control panel->systems->advanced system settings->environment variables->system variables

Matlab runtime
--------------

**For Linux**

- Download the latest matlab runtime and unzip

~~~~~~~~~~~~~~~~~~~~~
    sudo ./install
~~~~~~~~~~~~~~~~~~~~~

**For Windows**

- Download the latest matlab runtime and install

GDB
--------------

**Compile with GDB**

- Before using GDB, any source file needs to be
debugged must be compiled with -g flag     
~~~~~~~~~~~~~~~~~~~~~
   g++ -g ...
~~~~~~~~~~~~~~~~~~~~~

**Basic operations**

 - Start gdb
~~~~~~~~~~~~~~~~~~~~~
gdb executable_name 
~~~~~~~~~~~~~~~~~~~~~

- Set break point 
~~~~~~~~~~~~~~~~~~~~~
b simple.cpp:147 (break on a line)
b main (break upon entering a function)
b simple.cpp:147 if x==0 (break on condition)
~~~~~~~~~~~~~~~~~~~~~

- get all break points information
~~~~~~~~~~~~~~~~~~~~~
b info
~~~~~~~~~~~~~~~~~~~~~

- Disable/enable a break point. (break point ID can be found with "b info")
~~~~~~~~~~~~~~~~~~~~~
disable 1 
enable 1
~~~~~~~~~~~~~~~~~~~~~

- Run the program
~~~~~~~~~~~~~~~~~~~~~
r (run)
~~~~~~~~~~~~~~~~~~~~~

- Continue running after stop
~~~~~~~~~~~~~~~~~~~~~
 c (continue)
~~~~~~~~~~~~~~~~~~~~~

- step. Execute next line.
~~~~~~~~~~~~~~~~~~~~~
n (next)
~~~~~~~~~~~~~~~~~~~~~

- Step in. Go inside a function call
~~~~~~~~~~~~~~~~~~~~~
s (step)
~~~~~~~~~~~~~~~~~~~~~

- Step out of a function. Jump out of current function.
~~~~~~~~~~~~~~~~~~~~~
finish 
~~~~~~~~~~~~~~~~~~~~~

- Show variable value (temporary)
~~~~~~~~~~~~~~~~~~~~~
print x
print x.count
~~~~~~~~~~~~~~~~~~~~~

- Call a funciton
~~~~~~~~~~~~~~~~~~~~~
print x.count()
~~~~~~~~~~~~~~~~~~~~~

- Display/undisplay variable (permanent)
~~~~~~~~~~~~~~~~~~~~~
display x
undisplay x
~~~~~~~~~~~~~~~~~~~~~

- Get the type of the variable
~~~~~~~~~~~~~~~~~~~~~
ptype x 
~~~~~~~~~~~~~~~~~~~~~

- Check memory. 
~~~~~~~~~~~~~~~~~~~~~
x/256x ptr
~~~~~~~~~~~~~~~~~~~~~
This command shows 256 bytes presented as hexidecimal
starting at address of ptr. Format can be changed to
(d)ecimal, (u)nsigned, etc

- Quit gdb
~~~~~~~~~~~~~~~~~~~~~
quit
~~~~~~~~~~~~~~~~~~~~~

**Other**

- Find the source of "seg fault". At the most of the time, 
GDB will automatically stop at the place where the 
seg fault happened. This is a quick way to locate a seg fault.
~~~~~~~~~~~~~~~~~~~~~
gdb a.out
run
~~~~~~~~~~~~~~~~~~~~~

- cuda-gdb is capable of debugging cuda program running on 
the gpu and its usage is exactly the same as normal gdb.
~~~~~~~~~~~~~~~~~~~~~
cuda-gdb a.out
~~~~~~~~~~~~~~~~~~~~~




 

