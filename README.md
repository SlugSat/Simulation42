# Simulation 42 Custom Code Repository
The custom code we use in the 42 simulation. This repo does NOT contain the full source code for 42, only the files which we have modified. Clone this repo into the same folder as the 42 source code.

Last Edited:Feb 26,2020
Edited By: Juan Ayala Y4
The gitignore now ignores all original files within the 42 code. This means that once you download the 42 code, copy and paste the files within your local repo, merging but not replacing files.

To setup the code:
1. Download 42
2. Clone repo to your local computer
3. Copy and Paste 42 code into local repo
	-When trying to paste, your OS might tell you that there is already a file/folder there. If it asks to merge the folder, say yes. If it asks you to replace file, say no.
4. check git status
	-There should be no files that are commited or changed
	-If there are files changed type
	$git reset --hard

Note about .gitignore:
git ignore will ignore certian folders within the original 42 code as well as files (look in .gitignore to see the files).
This is to keep the repo with only the files we have changed or added. However, we could just have the whole 42 code, but this
is not what last years team wanted. So I will continue with that as well as make it easy by typing git add. everytime. 
There are some things you need to consider
1. If you make a change to any of these files or add something to the folders, they will not be added to the remote repo.
2. Any new files or folders introduced in the new versions of 42 will be added to the repo if the .gitignore is not updated
    
