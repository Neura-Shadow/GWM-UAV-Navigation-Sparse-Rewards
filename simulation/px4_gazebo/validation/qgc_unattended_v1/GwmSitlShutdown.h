#pragma once
// Opt-in Linux SITL integration. Normal QGC builds and interactive closes are unchanged.
#include <QtCore/QCoreApplication>
#include <QtCore/QCryptographicHash>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QSysInfo>
#include <QtCore/QVariant>
#include <cstdio>
#include <atomic>
#include <cerrno>
#if defined(GWM_QGC_SITL_SHUTDOWN_BUILD) && defined(Q_OS_LINUX)
#include "GwmSitlBuildIdentity.h"
#include <fcntl.h>
#include <ifaddrs.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#endif

namespace GwmSitlShutdown {
inline constexpr auto Mode = "isolated-sitl-active-connection-v1";
inline std::atomic<quint64> outboundCount{0}, commandCount{0};
inline std::atomic<bool> auditHealthy{true};
inline bool compiled() {
#if defined(GWM_QGC_SITL_SHUTDOWN_BUILD) && defined(Q_OS_LINUX)
    return true;
#else
    return false;
#endif
}
inline void audit(const char* stage, const QJsonObject& details = {}) {
    if (!compiled()) return;
    QJsonObject row = details; row["stage"] = QString::fromLatin1(stage);
    const QByteArray line = "GWM_SITL_SHUTDOWN_V1:" + QJsonDocument(row).toJson(QJsonDocument::Compact) + '\n';
    // Direct stderr is retained by the independent owner; do not rely on a queued Qt logger.
    if (std::fwrite(line.constData(), 1, size_t(line.size()), stderr) != size_t(line.size())
        || std::fflush(stderr) != 0) auditHealthy = false;
}
inline bool armed() {
    return compiled() && QCoreApplication::instance()
        && QCoreApplication::instance()->property("_gwmSitlTermVerified").toBool();
}
inline void stage(const char* name) { if (armed()) audit(name); }
inline QByteArray readBounded(const QString& path, qint64 max = 131072) {
    QFile file(path); if (!file.open(QIODevice::ReadOnly)) return {};
    QByteArray raw = file.read(max + 1); return raw.size() <= max ? raw : QByteArray{};
}
inline bool receiptFieldsValid(const QJsonObject& row, qint64 nowNs) {
    const double stamp = row.value("observed_monotonic_ns").toDouble(-1);
    return row.value("contract").toString() == "gwm-QGC-SITL-shutdown-ownership-v1"
        && row.value("mode").toString() == QLatin1String(Mode)
        && row.value("owner_pid").toInt(-1) == 1
        && row.value("health_verified").isBool() && row.value("health_verified").toBool()
        && row.value("landed").isBool() && row.value("landed").toBool()
        && row.value("arming_state").isDouble() && row.value("arming_state").toInt(-1) == 1
        && stamp >= 0 && stamp <= double(nowNs) && double(nowNs) - stamp <= 2000000000.;
}
inline bool verifyOwnership() {
#if defined(GWM_QGC_SITL_SHUTDOWN_BUILD) && defined(Q_OS_LINUX)
    if (qgetenv("GWM_QGC_SHUTDOWN_MODE") != Mode || getppid() != 1
        || getsid(0) != getpid() || getpgrp() != getpid()) return false;
    const QString path = QString::fromUtf8(qgetenv("GWM_QGC_SITL_SHUTDOWN_RECEIPT"));
    if (!QDir::isAbsolutePath(path) || QFileInfo(path).isSymLink()
        || QFileInfo(path).fileName() != "qgc-shutdown-ownership.json"
        || QDir(QFileInfo(path).absolutePath()).filePath("rootfs") != QDir::currentPath()) return false;
    int fd = ::open(QFile::encodeName(path).constData(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) return false;
    struct stat info{}; const bool owned = ::fstat(fd, &info) == 0 && info.st_uid == getuid()
        && (info.st_mode & 0777) == 0600 && S_ISREG(info.st_mode); ::close(fd);
    if (!owned) return false;
    const auto doc = QJsonDocument::fromJson(readBounded(path, 16384));
    if (!doc.isObject()) return false;
    const auto row = doc.object(); timespec now{};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0
        || !receiptFieldsValid(row, qint64(now.tv_sec) * 1000000000 + now.tv_nsec)
        || row.value("qgc_pid").toInt(-1) != getpid()
        || row.value("uid").toInt(-1) != int(getuid())) return false;
    const QByteArray stat = readBounded("/proc/1/stat");
    const auto fields = stat.mid(stat.lastIndexOf(')') + 2).split(' ');
    if (fields.size() < 20 || fields[19] != row.value("owner_start_ticks").toString().toUtf8()) return false;
    for (const auto* name : {"pid", "net", "mnt", "user"}) {
        auto link = [](const QString& file) {
            char target[256]; const auto n = ::readlink(QFile::encodeName(file).constData(), target, sizeof(target));
            return n > 0 && n < qint64(sizeof(target)) ? QString::fromLocal8Bit(target, n) : QString{};
        };
        const QString ns = link(QStringLiteral("/proc/self/ns/") + QLatin1String(name));
        if (ns.isEmpty() || ns != link(QStringLiteral("/proc/1/ns/") + QLatin1String(name))
            || ns != row.value(QStringLiteral("namespace_") + QLatin1String(name)).toString()) return false;
    }
    const auto mapping = readBounded("/proc/self/uid_map").simplified().split(' ');
    if (mapping.size() != 3 || mapping[0].toUInt() != getuid() || mapping[2] != "1") return false;
    ifaddrs* interfaces = nullptr; if (getifaddrs(&interfaces) != 0) return false;
    bool loopback = false, isolated = true;
    for (auto* entry = interfaces; entry; entry = entry->ifa_next) {
        if (QByteArray(entry->ifa_name) != "lo") isolated = false; else loopback = true;
    }
    freeifaddrs(interfaces); if (!isolated || !loopback) return false;
    const QString script = row.value("owner_script").toString();
    const QByteArray expected = GWM_QGC_OWNER_SOURCE_SHA256;
    const QByteArray source = readBounded(script);
    const auto ownerArgs = readBounded("/proc/1/cmdline").split('\0');
    if (expected.isEmpty() || row.value("owner_script_sha256").toString().toUtf8() != expected || source.isEmpty()
        || QCryptographicHash::hash(source, QCryptographicHash::Sha256).toHex() != expected
        || ownerArgs.size() < 2 || ownerArgs[1] != script.toUtf8()) return false;
    const QByteArray lockPath = QFile::encodeName(row.value("lock_path").toString());
    struct stat lock{}, held{};
    if (::stat(lockPath.constData(), &lock) != 0 || ::stat("/proc/1/fd/9", &held) != 0
        || lock.st_dev != held.st_dev || lock.st_ino != held.st_ino) return false;
    int lockFd = ::open(lockPath.constData(), O_RDWR | O_CLOEXEC); if (lockFd < 0) return false;
    const bool locked = ::flock(lockFd, LOCK_EX | LOCK_NB) != 0 && (errno == EWOULDBLOCK || errno == EAGAIN);
    ::close(lockFd); return locked;
#else
    return false;
#endif
}
inline void onTerm() {
    if (!compiled() || qgetenv("GWM_QGC_SHUTDOWN_MODE").isEmpty()) return;
    const bool verified = verifyOwnership();
    QCoreApplication::instance()->setProperty("_gwmSitlTermVerified", verified);
    audit(verified ? "TERM_ownership_verified" : "TERM_ownership_rejected");
}
inline void outboundAttempt(const QByteArray& raw) {
    if (!compiled() || qgetenv("GWM_QGC_AUDIT_NATIVE_ATTEMPTS") != "v1") return;
    // Observation only: every byte is retained, including forwarding and retries.
    const auto id = ++outboundCount;
    for (qsizetype offset = 0; offset < raw.size(); offset += 1024)
        audit("outbound_attempt", {{"attempt_id", QString::number(id)}, {"offset", double(offset)}, {"total_bytes", double(raw.size())},
            {"raw_hex", QString::fromLatin1(raw.mid(offset, 1024).toHex())}});
}
template<class... Values> inline void commandAttempt(int command, int targetComponent, Values... values) {
    if (!compiled() || qgetenv("GWM_QGC_AUDIT_NATIVE_ATTEMPTS") != "v1") return;
    const auto id = ++commandCount;
    QJsonArray parameters;
    auto append = [&parameters](auto value) {
        parameters.append(QJsonObject{{"native_bytes", int(sizeof(value))},
            {"raw_hex", QString::fromLatin1(QByteArray(reinterpret_cast<const char*>(&value), sizeof(value)).toHex())}});
    };
    (append(values), ...);
    audit("command_attempt", {{"attempt_id", QString::number(id)}, {"command", command}, {"target_component", targetComponent},
        {"byte_order", QSysInfo::ByteOrder == QSysInfo::LittleEndian ? "little" : "big"}, {"parameters", parameters}});
}
inline void auditTotals() {
    if (armed()) audit("native_attempt_audit_final", {{"outbound_attempts", QString::number(outboundCount.load())},
        {"command_attempts", QString::number(commandCount.load())}, {"writer_healthy", auditHealthy.load()}});
}
} // namespace GwmSitlShutdown
