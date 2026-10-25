#include "GwmSitlShutdown.h"
#include <QtCore/QCoreApplication>
#include <cassert>
#include <iostream>
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    const qint64 now=5000000000;
    QJsonObject valid{{"contract","gwm-QGC-SITL-shutdown-ownership-v1"},
        {"mode",GwmSitlShutdown::Mode},{"owner_pid",1},{"health_verified",true},
        {"landed",true},{"arming_state",1},{"observed_monotonic_ns",double(now)}};
    assert(GwmSitlShutdown::receiptFieldsValid(valid,now));
    for (const auto& key : {"contract","mode","owner_pid","health_verified","landed","arming_state","observed_monotonic_ns"}) {
        auto row=valid;row.remove(key);assert(!GwmSitlShutdown::receiptFieldsValid(row,now));
    }
    for (const auto& key : {"health_verified","landed"}) {
        auto row=valid;row[key]="true";assert(!GwmSitlShutdown::receiptFieldsValid(row,now));
        row[key]=false;assert(!GwmSitlShutdown::receiptFieldsValid(row,now));
    }
    auto old=valid;old["observed_monotonic_ns"]=double(now-2000000001);assert(!GwmSitlShutdown::receiptFieldsValid(old,now));
    old["observed_monotonic_ns"]=double(now+1);assert(!GwmSitlShutdown::receiptFieldsValid(old,now));
    assert(!GwmSitlShutdown::verifyOwnership()); // ordinary non-owned test process cannot arm.
    GwmSitlShutdown::onTerm();assert(!GwmSitlShutdown::armed());
    std::cout << "receipt type/freshness guards and non-owned-process rejection passed\n";
}
