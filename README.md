# YH-Project

## 221205
### git clone
``` bash
$ git clone [REPO_URL] [DIR]
# [REPO_URL]에는 클론해올 저장소의 주소를 지정해준다.
# [DIR] 인자는 저장소를 로컬에 복제할 위치를 지정한다. 생략 가능
$ git clone https://github.com/heew-d/YH-Project.git
Cloning into 'YH-Project'...
remote: Enumerating objects: 6, done.
remote: Counting objects: 100% (6/6), done.
remote: Compressing objects: 100% (3/3), done.
remote: Total 6 (delta 0), reused 3 (delta 0), pack-reused 0
Receiving objects: 100% (6/6), done.

# 저장소 복제가 성공적으로 진행됨. 디렉터리에 들어가보면 Git 저장소를 관리하는 .git 디렉터리와 README.md 파일이 생성되어 있음

$ cd YH-Project
$ ls -al
total 16
drwxr-xr-x   5 heewon  staff   160 12  5 20:02 .
drwxr-x---+ 44 heewon  staff  1408 12  5 20:02 ..
drwxr-xr-x  12 heewon  staff   384 12  5 20:02 .git
-rw-r--r--   1 heewon  staff    12 12  5 20:02 README.md
-rw-r--r--   1 heewon  staff    22 12  5 20:02 test.py
``` 

### 개발
``` bash
$ git add .
$ git commit -m "commit 메시지"
``` 

### Branch 생성
``` bash
$ git checkout -b [브랜치명] <!-- []제외하고 작성 -->
``` 

### push
``` bash
$ git push origin [브랜치명] <!-- []제외하고 작성 -->
``` 