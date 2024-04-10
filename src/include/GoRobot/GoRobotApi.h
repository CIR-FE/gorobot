#ifndef GO_ROBOT_API_H
#define GO_ROBOT_API_H

#include <kApi/kApi.h>

#if defined (GO_EMIT)
#    define GoFx(TYPE)    kExportFx(TYPE)
#    define GoCx(TYPE)    kExportCx(TYPE)
#    define GoDx(TYPE)    kExportDx(TYPE)
#else
#    define GoFx(TYPE)    kImportFx(TYPE)
#    define GoCx(TYPE)    kImportCx(TYPE)
#    define GoDx(TYPE)    kImportDx(TYPE)
#endif

#endif // !GO_ROBOT_API_H
