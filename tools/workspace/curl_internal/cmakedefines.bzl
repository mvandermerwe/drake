# The upstream lib/curl_config.h.cmake has a bajillion settings that it
# measures from the host platform. In order to have a reproducible build,
# we set them manually for Drake rather than measuring them.

# In our package.BUILD.bazel we invoke the cmake_configure_file() bazel macro
# in strict mode using these definitions, which means that any missing or
# extra definitions here will result in a build failure.

# Curl uses special-purpose CMake macros to set the SIZEOF_... variables, which
# our cmake_configure_file() bazel macro doesn't support. Therefore, we'll set
# these sizes via the preprocessor directly. (Drake only supports building on
# 64-bit platforms.) Keep this list alpha-sorted.
CPP_DEFINES = [
    "SIZEOF_CURL_OFF_T=8",
    "SIZEOF_INT=4",
    "SIZEOF_LONG=8",
    "SIZEOF_OFF_T=8",
    "SIZEOF_SIZE_T=8",
    "SIZEOF_TIME_T=8",
]

# These are the Apple-specific CMake definitions we want to enable for Drake.
# Keep this list alpha-sorted.
_APPLE_CMAKE_DEFINES = [
    "HAVE_FSETXATTR_6",
    "HAVE_SETMODE",
    "HAVE_SYS_FILIO_H",
    "HAVE_SYS_SOCKIO_H",
]

# These are the Linux-specific CMake definitions we want to enable for Drake.
# Keep this list alpha-sorted.
_LINUX_CMAKE_DEFINES = [
    "HAVE_CLOCK_GETTIME_MONOTONIC",
    "HAVE_FSETXATTR_5",
    "HAVE_GETHOSTBYNAME_R",
    "HAVE_GETHOSTBYNAME_R_6",
    "HAVE_MEMRCHR",
    "HAVE_MSG_NOSIGNAL",
    "HAVE_TERMIOS_H",
]

# These are the settings we want to enable for Drake. A few of them towards the
# bottom are conditional for Apple vs Linux. Keep this list alpha-sorted.
CMAKE_DEFINES = [
    "CURL_DISABLE_ALTSVC",
    "CURL_DISABLE_AWS",
    "CURL_DISABLE_BASIC_AUTH",
    "CURL_DISABLE_BEARER_AUTH",
    "CURL_DISABLE_BINDLOCAL",
    "CURL_DISABLE_COOKIES",
    "CURL_DISABLE_DICT",
    "CURL_DISABLE_DIGEST_AUTH",
    "CURL_DISABLE_DOH",
    "CURL_DISABLE_FILE",
    "CURL_DISABLE_FORM_API",
    "CURL_DISABLE_FTP",
    "CURL_DISABLE_GETOPTIONS",
    "CURL_DISABLE_GOPHER",
    "CURL_DISABLE_HEADERS_API",
    "CURL_DISABLE_HSTS",
    "CURL_DISABLE_IMAP",
    "CURL_DISABLE_KERBEROS_AUTH",
    "CURL_DISABLE_LDAP",
    "CURL_DISABLE_LDAPS",
    "CURL_DISABLE_MQTT",
    "CURL_DISABLE_NEGOTIATE_AUTH",
    "CURL_DISABLE_NETRC",
    "CURL_DISABLE_NTLM",
    "CURL_DISABLE_OPENSSL_AUTO_LOAD_CONFIG",
    "CURL_DISABLE_PARSEDATE",
    "CURL_DISABLE_POP3",
    "CURL_DISABLE_PROGRESS_METER",
    "CURL_DISABLE_PROXY",
    "CURL_DISABLE_RTSP",
    "CURL_DISABLE_SMB",
    "CURL_DISABLE_SMTP",
    "CURL_DISABLE_TELNET",
    "CURL_DISABLE_TFTP",
    "ENABLE_IPV6",
    "HAVE_ALARM",
    "HAVE_ARPA_INET_H",
    "HAVE_ATOMIC",
    "HAVE_BASENAME",
    "HAVE_BOOL_T",
    "HAVE_CLOCK_GETTIME_MONOTONIC_RAW",
    "HAVE_DECL_FSEEKO",
    "HAVE_FCNTL",
    "HAVE_FCNTL_H",
    "HAVE_FCNTL_O_NONBLOCK",
    "HAVE_FNMATCH",
    "HAVE_FREEADDRINFO",
    "HAVE_FSEEKO",
    "HAVE_FSETXATTR",
    "HAVE_FTRUNCATE",
    "HAVE_GETADDRINFO",
    "HAVE_GETADDRINFO_THREADSAFE",
    "HAVE_GETEUID",
    "HAVE_GETHOSTNAME",
    "HAVE_GETIFADDRS",
    "HAVE_GETPEERNAME",
    "HAVE_GETPPID",
    "HAVE_GETPWUID",
    "HAVE_GETPWUID_R",
    "HAVE_GETRLIMIT",
    "HAVE_GETSOCKNAME",
    "HAVE_GETTIMEOFDAY",
    "HAVE_GMTIME_R",
    "HAVE_IFADDRS_H",
    "HAVE_IF_NAMETOINDEX",
    "HAVE_INET_NTOP",
    "HAVE_INET_PTON",
    "HAVE_IOCTL_FIONBIO",
    "HAVE_IOCTL_SIOCGIFADDR",
    "HAVE_LIBGEN_H",
    "HAVE_LIBZ",
    "HAVE_LOCALE_H",
    "HAVE_LONGLONG",
    "HAVE_NETDB_H",
    "HAVE_NETINET_IN_H",
    "HAVE_NETINET_TCP_H",
    "HAVE_NETINET_UDP_H",
    "HAVE_NET_IF_H",
    "HAVE_PIPE",
    "HAVE_POLL_FINE",
    "HAVE_POLL_H",
    "HAVE_POSIX_STRERROR_R",
    "HAVE_PWD_H",
    "HAVE_RECV",
    "HAVE_SCHED_YIELD",
    "HAVE_SELECT",
    "HAVE_SEND",
    "HAVE_SENDMSG",
    "HAVE_SETLOCALE",
    "HAVE_SETRLIMIT",
    "HAVE_SIGACTION",
    "HAVE_SIGINTERRUPT",
    "HAVE_SIGNAL",
    "HAVE_SIGSETJMP",
    "HAVE_SNPRINTF",
    "HAVE_SOCKADDR_IN6_SIN6_SCOPE_ID",
    "HAVE_SOCKET",
    "HAVE_SOCKETPAIR",
    "HAVE_STDATOMIC_H",
    "HAVE_STDBOOL_H",
    "HAVE_STRCASECMP",
    "HAVE_STRDUP",
    "HAVE_STRERROR_R",
    "HAVE_STRINGS_H",
    "HAVE_STRTOK_R",
    "HAVE_STRTOLL",
    "HAVE_STRUCT_SOCKADDR_STORAGE",
    "HAVE_STRUCT_TIMEVAL",
    "HAVE_SUSECONDS_T",
    "HAVE_SYS_IOCTL_H",
    "HAVE_SYS_PARAM_H",
    "HAVE_SYS_POLL_H",
    "HAVE_SYS_RESOURCE_H",
    "HAVE_SYS_SELECT_H",
    "HAVE_SYS_SOCKET_H",
    "HAVE_SYS_STAT_H",
    "HAVE_SYS_TIME_H",
    "HAVE_SYS_TYPES_H",
    "HAVE_SYS_UN_H",
    "HAVE_SYS_WAIT_H",
    "HAVE_UNISTD_H",
    "HAVE_UTIME",
    "HAVE_UTIME_H",
    "HAVE_WRITABLE_ARGV",
    "OS=\"\"",
    "STDC_HEADERS",
] + [
    # We set these special-purpose CMake macros to blank.
    # Refer to CPP_DEFINES (above) for an explanation.
    "SIZEOF_CURL_SOCKET_T_CODE=",
    "SIZEOF_INT_CODE=",
    "SIZEOF_LONG_CODE=",
    "SIZEOF_LONG_LONG_CODE=",
    "SIZEOF_OFF_T_CODE=",
    "SIZEOF_CURL_OFF_T_CODE=",
    "SIZEOF_SIZE_T_CODE=",
    "SIZEOF_TIME_T_CODE=",
] + select({
    "@drake//tools/cc_toolchain:apple": _APPLE_CMAKE_DEFINES,
    "@drake//tools/cc_toolchain:linux": _LINUX_CMAKE_DEFINES,
    "//conditions:default": [],
})

# These are the settings we want to disable for Drake on all platforms.
# Keep this list alpha-sorted.
CMAKE_UNDEFINES = [
    "CURL_CA_BUNDLE",
    "CURL_CA_FALLBACK",
    "CURL_CA_PATH",
    "CURL_DEFAULT_SSL_BACKEND",
    "CURL_DISABLE_HTTP",
    "CURL_DISABLE_LIBCURL_OPTION",
    "CURL_DISABLE_MIME",
    "CURL_DISABLE_SOCKETPAIR",
    "CURL_DISABLE_VERBOSE_STRINGS",
    "CURL_EXTERN_SYMBOL",
    "CURL_WITH_MULTI_SSL",
    "HAVE_ADDRESS_FAMILY",
    "HAVE_ARC4RANDOM",
    "HAVE_BROTLI",
    "HAVE_BUILTIN_AVAILABLE",
    "HAVE_CLOSESOCKET",
    "HAVE_GETHOSTBYNAME_R_3",
    "HAVE_GETHOSTBYNAME_R_5",
    "HAVE_GETPASS_R",
    "HAVE_GLIBC_STRERROR_R",
    "HAVE_GNUTLS_SRP",
    "HAVE_GSSAPI",
    "HAVE_GSSAPI_GSSAPI_GENERIC_H",
    "HAVE_GSSAPI_GSSAPI_H",
    "HAVE_GSSAPI_GSSAPI_KRB5_H",
    "HAVE_GSSGNU",
    "HAVE_IDN2_H",
    "HAVE_IDNA_STRERROR",
    "HAVE_IOCTLSOCKET",
    "HAVE_IOCTLSOCKET_CAMEL",
    "HAVE_IOCTLSOCKET_CAMEL_FIONBIO",
    "HAVE_IOCTLSOCKET_FIONBIO",
    "HAVE_IO_H",
    "HAVE_LBER_H",
    "HAVE_LDAP_H",
    "HAVE_LDAP_SSL",
    "HAVE_LDAP_SSL_H",
    "HAVE_LDAP_URL_PARSE",
    "HAVE_LIBIDN2",
    "HAVE_LIBSOCKET",
    "HAVE_LIBSSH2",
    "HAVE_LINUX_TCP_H",
    "HAVE_MACH_ABSOLUTE_TIME",
    "HAVE_OLD_GSSMIT",
    "HAVE_OPENSSL_SRP",
    "HAVE_PTHREAD_H",
    "HAVE_QUICHE_CONN_SET_QLOG_FD",
    "HAVE_SA_FAMILY_T",
    "HAVE_SETSOCKOPT_SO_NONBLOCK",
    "HAVE_SSL_SET0_WBIO",
    "HAVE_STRCMPI",
    "HAVE_STRICMP",
    "HAVE_STROPTS_H",
    "HAVE_SYS_UTIME_H",
    "HAVE_TERMIO_H",
    "HAVE_UTIMES",
    "HAVE_ZSTD",
    "HAVE__FSEEKI64",
    "NEED_LBER_H",
    "NEED_MALLOC_H",
    "NEED_REENTRANT",
    "PACKAGE",
    "PACKAGE_BUGREPORT",
    "PACKAGE_NAME",
    "PACKAGE_STRING",
    "PACKAGE_TARNAME",
    "PACKAGE_VERSION",
    "RANDOM_FILE",
    "USE_ARES",
    "USE_BEARSSL",
    "USE_GNUTLS",
    "USE_LIBPSL",
    "USE_LIBSSH",
    "USE_LIBSSH2",
    "USE_MBEDTLS",
    "USE_MSH3",
    "USE_NGHTTP2",
    "USE_NGHTTP3",
    "USE_NGTCP2",
    "USE_OPENLDAP",
    "USE_OPENSSL",
    "USE_QUICHE",
    "USE_SCHANNEL",
    "USE_SECTRANSP",
    "USE_THREADS_POSIX",
    "USE_THREADS_WIN32",
    "USE_TLS_SRP",
    "USE_UNIX_SOCKETS",
    "USE_WEBSOCKETS",
    "USE_WIN32_CRYPTO",
    "USE_WIN32_IDN",
    "USE_WIN32_LARGE_FILES",
    "USE_WIN32_LDAP",
    "USE_WINDOWS_SSPI",
    "USE_WOLFSSL",
    "VERSION",
    "_FILE_OFFSET_BITS",
    "_LARGE_FILES",
    "_THREAD_SAFE",
    "const",
    "in_addr_t",
    "size_t",
    "ssize_t",
] + select({
    "@drake//tools/cc_toolchain:apple": _LINUX_CMAKE_DEFINES,
    "@drake//tools/cc_toolchain:linux": _APPLE_CMAKE_DEFINES,
    "//conditions:default": _LINUX_CMAKE_DEFINES + _APPLE_CMAKE_DEFINES,
})
