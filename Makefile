OMPL_ARCHIVE = ompl_r`svn info | grep Revision | sed s/Revision:\ //`.tar.gz

all:	ompl_archive

ompl_archive:
	echo Generating $(OMPL_ARCHIVE)
	-rm -f ompl_r*.tar.gz*
	tar -zcf $(OMPL_ARCHIVE) --exclude-vcs ompl
	md5sum $(OMPL_ARCHIVE) > $(OMPL_ARCHIVE).md5sum
