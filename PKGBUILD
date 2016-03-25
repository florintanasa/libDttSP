# Maintainer: Jeff Baitis <jeff@baitis.net>
pkgname=libdttsp-git
pkgver=dc4338d
pkgrel=2
pkgdesc="Software defined radio processing core library from the DTTS Microwave Society"
arch=('i686' 'x86_64')
url="https://sourceforge.net/projects/dttsp/"
license=('GPL-2')
groups=('')
depends=(
	'glibc' 
	'liblo' 
	'jack' 
	'fftw' 
	'db')
makedepends=('git' 'make') # 'bzr', 'git', 'mercurial' or 'subversion'
provides=("${pkgname}-git")
conflicts=("${pkgname}-git")
source=("${pkgname}::git://github.com/wd8rde/libDttSP")
md5sums=('SKIP')

# Please refer to the 'USING VCS SOURCES' section of the PKGBUILD man page for
# a description of each element in the source array.

pkgver() {
	pkgver_git
}

pkgver_git() {
    cd "${srcdir}/${pkgname}"
    local ver="$(git show | grep commit | awk '{print $2}' )"
    #printf "r%s" "${ver//[[:alpha:]]}"
    echo ${ver:0:7}
}

build() {
	cd "$srcdir/${pkgname}/"
	./bootstrap
	./configure --prefix=/usr
	make
}

check() {
	cd "$srcdir/${pkgname%-VCS}"
	make -k check
}

package() {
	cd "$srcdir/${pkgname%-VCS}"
	make DESTDIR="$pkgdir/" install
}
