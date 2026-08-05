#include "ti_vision/web_server.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ti_vision {
namespace {

constexpr std::size_t kMaxRequestBytes = 16384;

std::uint64_t steady_now_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

const char* mode_name(TxMode mode) {
    switch (mode) {
        case TxMode::Tracking: return "tracking";
        case TxMode::Manual: return "manual";
        case TxMode::Stopped: return "stopped";
    }
    return "stopped";
}

bool send_all(int fd, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t offset = 0;
    while (offset < size) {
        const ssize_t sent = send(fd, bytes + offset, size - offset, MSG_NOSIGNAL);
        if (sent > 0) {
            offset += static_cast<std::size_t>(sent);
        } else if (sent < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

void send_response(
    int fd,
    int status,
    const char* reason,
    const char* content_type,
    const void* body,
    std::size_t body_size,
    const std::string& extra_headers = {}) {
    std::ostringstream header;
    header << "HTTP/1.1 " << status << ' ' << reason << "\r\n"
           << "Content-Type: " << content_type << "\r\n"
           << "Content-Length: " << body_size << "\r\n"
           << "Cache-Control: no-store\r\n"
           << "X-Content-Type-Options: nosniff\r\n"
           << extra_headers
           << "Connection: close\r\n\r\n";
    const std::string text = header.str();
    if (send_all(fd, text.data(), text.size()) && body_size > 0) {
        send_all(fd, body, body_size);
    }
}

std::string url_decode(const std::string& input) {
    std::string output;
    output.reserve(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (input[i] == '+') {
            output.push_back(' ');
        } else if (input[i] == '%' && i + 2 < input.size()) {
            char* end = nullptr;
            const std::string hex = input.substr(i + 1, 2);
            const long value = std::strtol(hex.c_str(), &end, 16);
            if (end != hex.c_str() + 2) {
                return {};
            }
            output.push_back(static_cast<char>(value));
            i += 2;
        } else {
            output.push_back(input[i]);
        }
    }
    return output;
}

std::map<std::string, std::string> parse_form(const std::string& body) {
    std::map<std::string, std::string> values;
    std::size_t start = 0;
    while (start <= body.size()) {
        const std::size_t end = body.find('&', start);
        const std::string item = body.substr(start, end == std::string::npos ? end : end - start);
        const std::size_t separator = item.find('=');
        if (separator != std::string::npos) {
            values[url_decode(item.substr(0, separator))] = url_decode(item.substr(separator + 1));
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return values;
}

bool parse_bounded_float(const std::string& text, float& value) {
    try {
        std::size_t used = 0;
        const float parsed = std::stof(text, &used);
        if (used != text.size() || !std::isfinite(parsed) || parsed < -12.0F || parsed > 12.0F) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_target(const std::string& text, std::int8_t& value) {
    try {
        std::size_t used = 0;
        const int parsed = std::stoi(text, &used);
        if (used != text.size() || parsed < -12 || parsed > 12) {
            return false;
        }
        value = static_cast<std::int8_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

std::string json_error(const std::string& error) {
    return "{\"ok\":false,\"error\":\"" + error + "\"}";
}

const char kIndexHtml[] = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RDK 滚球视觉控制</title><style>
:root{color-scheme:dark;--bg:#090b0f;--panel:#12161d;--line:#29313c;--text:#f4f6f8;--muted:#8d98a8;--cyan:#3bdbc9;--yellow:#ffd166;--red:#ff5d6c;--blue:#6ba7ff}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.5 system-ui,"Microsoft YaHei",sans-serif}.shell{width:min(1080px,calc(100% - 28px));margin:24px auto 48px}.top{display:flex;justify-content:space-between;align-items:flex-end;gap:18px;margin-bottom:18px}h1{font-size:clamp(25px,4vw,40px);line-height:1;margin:0;letter-spacing:-.03em}.sub{color:var(--muted);margin-top:9px}.chips{display:flex;gap:8px;flex-wrap:wrap;justify-content:flex-end}.chip{border:1px solid var(--line);border-radius:999px;padding:5px 10px;color:var(--muted);font-size:12px}.chip.live{color:var(--cyan);border-color:#246c67}.chip.rec{color:var(--red);border-color:#8c3440}.grid{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(300px,.8fr);gap:16px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px}.stage{background:#050607;border-radius:9px;overflow:hidden;position:relative;aspect-ratio:20/3;min-height:112px;display:grid;place-items:center}canvas{width:100%;height:100%;image-rendering:auto}.empty{position:absolute;color:var(--muted)}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}.metric{background:#0d1117;border-radius:9px;padding:10px}.metric b{display:block;font-size:20px;font-variant-numeric:tabular-nums}.metric span,label,.hint{color:var(--muted);font-size:12px}.section+.section{border-top:1px solid var(--line);margin-top:16px;padding-top:16px}.section h2{font-size:15px;margin:0 0 10px}.row{display:flex;gap:8px;align-items:center}.number,select{background:#090c10;border:1px solid var(--line);color:var(--text);border-radius:8px;padding:9px;font:inherit}.number{width:90px}select{width:100%}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}button{border:1px solid var(--line);background:#1a2029;color:var(--text);border-radius:8px;padding:10px 12px;font-weight:650;cursor:pointer}button:hover{border-color:#566171}button.primary{background:var(--cyan);color:#06110f;border-color:var(--cyan)}button.record{background:#76222d;border-color:#a53b49}.wide{grid-column:1/-1}.stop{color:#fff;background:#641d28;border-color:#9d3341}.mode-line{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}.mode{font-weight:750}.error{color:var(--red);min-height:22px;margin-top:8px}.hint{margin:8px 0 0}.timeline{width:100%;margin-top:10px}.replayline{display:flex;justify-content:space-between;color:var(--muted);font-size:12px;margin-top:5px}@media(max-width:780px){.grid{grid-template-columns:1fr}.top{align-items:flex-start;flex-direction:column}.chips{justify-content:flex-start}.metrics{grid-template-columns:1fr 1fr}}
</style></head><body><main class="shell">
<div class="top"><div><h1>RDK 滚球视觉控制</h1><div class="sub">目标控制 · 识别画面 · 完整测试回放</div></div><div class="chips"><span class="chip" id="link">连接中</span><span class="chip" id="recChip">未记录</span><span class="chip">640×96 / 8 FPS</span></div></div>
<div class="grid"><section class="card"><div class="stage"><div class="empty" id="empty">等待识别画面</div><canvas id="view" width="640" height="96"></canvas></div><div class="metrics"><div class="metric"><b id="measured">—</b><span>识别位置 / cm</span></div><div class="metric"><b id="rawSpeed">—</b><span>原始差分速度 / cm/s</span></div><div class="metric"><b id="measuredSpeed">—</b><span>α-β 观测速度 / cm/s</span></div><div class="metric"><b id="sent">—</b><span>发送位置 / cm</span></div><div class="metric"><b id="sentSpeed">—</b><span>发送速度 / cm/s</span></div><div class="metric"><b id="score">—</b><span>置信度</span></div><div class="metric"><b id="fps">—</b><span>推理 FPS</span></div></div><p class="hint" id="detail">等待状态数据</p>
<div class="section"><h2>测试记录与回放</h2><div class="buttons"><button class="record" id="recordStart">开始完整记录</button><button id="recordStop">停止并保存</button></div><div class="row" style="margin-top:12px"><select id="records"><option value="">暂无记录</option></select></div><div class="buttons"><button id="play">播放选中记录</button><button id="pause">暂停</button><button class="wide" id="returnLive">返回实时画面</button></div><input class="timeline" id="timeline" type="range" min="0" max="0" value="0"><div class="replayline"><span id="replayTime">实时</span><span id="recordInfo">浏览器回放不占用 BPU</span></div></div></section>
<aside class="card"><div class="mode-line"><span>发送模式</span><span class="mode" id="mode">自动识别</span></div><div class="section"><h2>目标位置控制</h2><div class="row"><input id="target" class="number" type="number" min="-12" max="12" step="1" value="0"><span>cm / 指令</span></div><div class="buttons"><button class="primary wide" id="track">应用目标并自动发送</button><button data-target="0">中心 0</button><button data-target="1">第三题 +5→-5</button><button data-target="5">固定 +5</button><button data-target="-5">固定 -5</button></div><p class="hint">协议为有符号整数：0=中心；1=第三题自动斜坡；其他值=固定目标 cm。</p></div>
<div class="section"><h2>手动位置测试</h2><div class="row"><input id="manual" class="number" type="number" min="-12" max="12" step="0.1" value="0"><span>cm</span></div><div class="buttons"><button id="manualSend">发送手动位置</button><button id="zero">手动发 0</button><button class="stop wide" id="stop">立即停发</button></div><p class="hint">手动模式仅用于方向/PID辨识；目标指令仍随包发送。</p></div><div class="error" id="error"></div></aside></div></main>
<script>
const $=id=>document.getElementById(id),canvas=$('view'),ctx=canvas.getContext('2d'),image=ctx.createImageData(640,96);let state=null,busy=false,controlsInit=false,recordings=[],replay={active:false,playing:false,info:null,index:0,timer:null};
const f=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):'—';
function draw(y,m){if(y.length!==640*96)return;for(let i=0,j=0;i<y.length;i++,j+=4){const v=y[i];image.data[j]=v;image.data[j+1]=v;image.data[j+2]=v;image.data[j+3]=255}ctx.putImageData(image,0,0);$('empty').style.display='none';if(m?.detected){ctx.strokeStyle='#3bdbc9';ctx.lineWidth=2;ctx.strokeRect(m.cx-m.box_width/2,m.cy-m.box_height/2,m.box_width,m.box_height)}if(m&&Number.isFinite(m.target_x)){ctx.strokeStyle='#ffd166';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(m.target_x,0);ctx.lineTo(m.target_x,96);ctx.stroke()}}
async function getStatus(){try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error();state=await r.json();$('link').textContent='已连接';$('link').className='chip live';$('mode').textContent={tracking:'自动识别',manual:'手动位置',stopped:'已停发'}[state.tx_mode]||state.tx_mode;if(!replay.active){$('measured').textContent=state.detected?f(state.measured_cm):'丢失';$('rawSpeed').textContent=state.detected?f(state.raw_speed_cm_s):'—';$('measuredSpeed').textContent=state.detected?f(state.measured_speed_cm_s):'—';$('sent').textContent=state.transmitting?f(state.sent_cm):'停发';$('sentSpeed').textContent=state.transmitting?f(state.sent_speed_cm_s):'停发';$('score').textContent=state.detected?f(state.score,3):'—';$('fps').textContent=f(state.infer_fps,1)}$('detail').textContent=`位置/速度同源线性 α-β τ=40 ms · UART 500 Hz 帧间线性外推 · 解码 ${f(state.decode_fps,1)} FPS · 丢弃旧帧 ${state.dropped_frames} · 超期 ${state.tx_deadline_misses}`;$('recChip').textContent=state.recording?`记录中 ${state.active_recording}`:'未记录';$('recChip').className=state.recording?'chip rec':'chip';if(!controlsInit){$('target').value=state.target_control;$('manual').value=f(state.manual_cm,1);controlsInit=true}}catch(e){$('link').textContent='连接断开';$('link').className='chip'}}
async function liveFrame(){if(busy||replay.active)return;busy=true;try{const r=await fetch('/api/frame',{cache:'no-store'});if(r.status===204)return;if(!r.ok)throw Error();draw(new Uint8Array(await r.arrayBuffer()),state)}catch(e){}finally{busy=false}}
async function control(mode,target=$('target').value,manual=$('manual').value){$('error').textContent='';const body=new URLSearchParams({mode,target_control:String(target),manual_cm:String(manual)});try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const d=await r.json();if(!r.ok)throw Error(d.error||'控制失败');await getStatus()}catch(e){$('error').textContent=e.message}}
async function record(action){$('error').textContent='';try{const r=await fetch('/api/recording',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({action})});const d=await r.json();if(!r.ok)throw Error(d.error||'记录操作失败');await getStatus();setTimeout(loadRecordings,500)}catch(e){$('error').textContent=e.message}}
async function loadRecordings(){try{const r=await fetch('/api/recordings',{cache:'no-store'});recordings=await r.json();const old=$('records').value;$('records').innerHTML=recordings.length?'':'<option value="">暂无记录</option>';for(const x of recordings){const o=document.createElement('option');o.value=x.id;o.textContent=`${x.id} · ${(x.duration_ms/1000).toFixed(1)}s · ${x.frame_count}帧`;$('records').appendChild(o)}if(recordings.some(x=>x.id===old))$('records').value=old}catch(e){}}
function replayMeta(r){return{detected:r.headers.get('X-Detected')==='1',cx:+r.headers.get('X-Cx'),cy:+r.headers.get('X-Cy'),box_width:+r.headers.get('X-Box-Width'),box_height:+r.headers.get('X-Box-Height'),score:+r.headers.get('X-Score'),measured_cm:+r.headers.get('X-Measured-Cm'),sent_cm:+r.headers.get('X-Sent-Cm'),target_control:+r.headers.get('X-Target-Control'),elapsed_ms:+r.headers.get('X-Elapsed-Ms')}}
async function showReplay(index){if(!replay.info)return;replay.index=Math.max(0,Math.min(index,replay.info.frame_count-1));const r=await fetch(`/api/replay?id=${encodeURIComponent(replay.info.id)}&index=${replay.index}`,{cache:'no-store'});if(!r.ok)return;const m=replayMeta(r);draw(new Uint8Array(await r.arrayBuffer()),m);$('timeline').value=replay.index;$('replayTime').textContent=`回放 ${(m.elapsed_ms/1000).toFixed(2)} / ${(replay.info.duration_ms/1000).toFixed(2)} s`;$('measured').textContent=m.detected?f(m.measured_cm):'丢失';$('rawSpeed').textContent='—';$('measuredSpeed').textContent='—';$('sent').textContent=f(m.sent_cm);$('sentSpeed').textContent='—';$('score').textContent=m.detected?f(m.score,3):'—';$('recordInfo').textContent=`目标指令 ${m.target_control} · 第 ${replay.index+1}/${replay.info.frame_count} 帧`}
function stopPlay(){replay.playing=false;if(replay.timer)clearTimeout(replay.timer);replay.timer=null}
async function tick(){if(!replay.playing||!replay.info)return;await showReplay(replay.index);if(replay.index+1>=replay.info.frame_count){stopPlay();return}replay.index++;replay.timer=setTimeout(tick,1000/replay.info.fps)}
function startReplay(){const info=recordings.find(x=>x.id===$('records').value);if(!info)return;stopPlay();replay={active:true,playing:true,info,index:0,timer:null};$('timeline').max=Math.max(0,info.frame_count-1);tick()}
function returnLive(){stopPlay();replay.active=false;replay.info=null;$('timeline').value=0;$('timeline').max=0;$('replayTime').textContent='实时';$('recordInfo').textContent='浏览器回放不占用 BPU'}
$('track').onclick=()=>control('tracking');$('manualSend').onclick=()=>control('manual');$('zero').onclick=()=>{$('manual').value=0;control('manual')};$('stop').onclick=()=>control('stopped');document.querySelectorAll('[data-target]').forEach(b=>b.onclick=()=>{$('target').value=b.dataset.target;control('tracking')});$('recordStart').onclick=()=>record('start');$('recordStop').onclick=()=>record('stop');$('play').onclick=startReplay;$('pause').onclick=()=>{if(!replay.active)return;if(replay.playing)stopPlay();else{replay.playing=true;tick()}};$('returnLive').onclick=returnLive;$('timeline').oninput=e=>{if(replay.active){stopPlay();showReplay(+e.target.value)}};
getStatus();liveFrame();loadRecordings();setInterval(getStatus,250);setInterval(liveFrame,125);setInterval(loadRecordings,2000);
</script></body></html>)HTML";

}  // namespace

WebServer::WebServer(
    int port,
    int preview_fps,
    std::string recordings_directory,
    float zero_px,
    float cm_per_px,
    std::int8_t initial_target_control)
    : port_(port),
      preview_fps_(preview_fps),
      zero_px_(zero_px),
      cm_per_px_(cm_per_px),
      target_control_(initial_target_control),
      recordings_(std::move(recordings_directory), 640, 96, preview_fps) {
    if (port <= 0 || port > 65535 || preview_fps <= 0 || preview_fps > 30 ||
        !std::isfinite(zero_px) || !std::isfinite(cm_per_px) || cm_per_px <= 0.0F) {
        throw std::invalid_argument("invalid web server configuration");
    }
}

WebServer::~WebServer() {
    stop();
}

void WebServer::start() {
    if (running_.exchange(true)) {
        return;
    }
    recordings_.start_worker();
    listen_fd_ = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (listen_fd_ < 0) {
        recordings_.shutdown();
        running_ = false;
        throw std::runtime_error(std::string("web socket: ") + std::strerror(errno));
    }
    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port_));
    if (bind(listen_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
        listen(listen_fd_, 8) != 0) {
        const std::string message = std::string("web bind/listen: ") + std::strerror(errno);
        close(listen_fd_);
        listen_fd_ = -1;
        recordings_.shutdown();
        running_ = false;
        throw std::runtime_error(message);
    }
    thread_ = std::thread(&WebServer::run, this);
}

void WebServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (listen_fd_ >= 0) {
        shutdown(listen_fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }
    recordings_.shutdown();
}

TxMode WebServer::tx_mode() const noexcept {
    return static_cast<TxMode>(tx_mode_.load(std::memory_order_relaxed));
}

std::int8_t WebServer::target_control() const noexcept {
    return target_control_.load(std::memory_order_relaxed);
}

float WebServer::manual_cm() const noexcept {
    return manual_cm_.load(std::memory_order_relaxed);
}

void WebServer::update(
    const std::uint8_t* y,
    std::size_t y_size,
    int width,
    int height,
    const Detection& detection,
    float measured_cm,
    float raw_speed_cm_s,
    float measured_speed_cm_s,
    float sent_cm,
    float sent_speed_cm_s,
    bool transmitting,
    double decode_fps,
    double infer_fps,
    std::uint64_t decoded_frames,
    std::uint64_t inferred_frames,
    std::uint64_t dropped_frames,
    std::uint64_t decoder_resets,
    std::uint64_t tx_frames,
    std::uint64_t tx_deadline_misses) {
    const std::uint64_t now = steady_now_ns();
    const std::uint64_t period = 1000000000ULL / static_cast<std::uint64_t>(preview_fps_);
    bool sampled = false;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (now >= next_preview_ns_ && y != nullptr &&
            y_size == static_cast<std::size_t>(width * height)) {
            snapshot_.y.assign(y, y + y_size);
            snapshot_.width = width;
            snapshot_.height = height;
            next_preview_ns_ = now + period;
            sampled = true;
        }
        snapshot_.detection = detection;
        snapshot_.measured_cm = measured_cm;
        snapshot_.raw_speed_cm_s = raw_speed_cm_s;
        snapshot_.measured_speed_cm_s = measured_speed_cm_s;
        snapshot_.sent_cm = sent_cm;
        snapshot_.sent_speed_cm_s = sent_speed_cm_s;
        snapshot_.transmitting = transmitting;
        snapshot_.decode_fps = decode_fps;
        snapshot_.infer_fps = infer_fps;
        snapshot_.decoded_frames = decoded_frames;
        snapshot_.inferred_frames = inferred_frames;
        snapshot_.dropped_frames = dropped_frames;
        snapshot_.decoder_resets = decoder_resets;
        snapshot_.tx_frames = tx_frames;
        snapshot_.tx_deadline_misses = tx_deadline_misses;
        snapshot_.updated_ns = now;
    }
    if (sampled) {
        recordings_.enqueue(
            y, y_size, measured_cm, sent_cm, target_control(), detection);
    }
}

void WebServer::run() {
    sched_param idle{};
    pthread_setschedparam(pthread_self(), SCHED_IDLE, &idle);
    while (running_.load()) {
        sockaddr_in client_address{};
        socklen_t client_size = sizeof(client_address);
        const int client = accept4(
            listen_fd_, reinterpret_cast<sockaddr*>(&client_address), &client_size, SOCK_CLOEXEC);
        if (client < 0) {
            if (running_.load() && errno != EINTR) {
                std::cerr << "web accept: " << std::strerror(errno) << '\n';
            }
            continue;
        }
        timeval timeout{0, 500000};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        handle_client(client);
        close(client);
    }
}

void WebServer::handle_client(int client_fd) {
    std::string request;
    request.reserve(4096);
    std::size_t expected_size = 0;
    while (request.size() < kMaxRequestBytes) {
        char buffer[4096];
        const std::size_t remaining = kMaxRequestBytes - request.size();
        const ssize_t received = recv(client_fd, buffer, std::min(sizeof(buffer), remaining), 0);
        if (received <= 0) {
            break;
        }
        request.append(buffer, static_cast<std::size_t>(received));
        const std::size_t current_header_end = request.find("\r\n\r\n");
        if (current_header_end == std::string::npos) {
            continue;
        }
        if (expected_size == 0) {
            std::string lowercase_headers = request.substr(0, current_header_end);
            std::transform(
                lowercase_headers.begin(), lowercase_headers.end(), lowercase_headers.begin(),
                [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            const std::string key = "content-length:";
            const std::size_t length_at = lowercase_headers.find(key);
            std::size_t content_length = 0;
            if (length_at != std::string::npos) {
                const char* number = lowercase_headers.c_str() + length_at + key.size();
                content_length = static_cast<std::size_t>(std::strtoul(number, nullptr, 10));
            }
            expected_size = current_header_end + 4 + content_length;
            if (expected_size > kMaxRequestBytes) {
                break;
            }
        }
        if (request.size() >= expected_size) {
            break;
        }
    }
    const std::size_t header_end = request.find("\r\n\r\n");
    if (header_end == std::string::npos) {
        static const char body[] = "bad request";
        send_response(client_fd, 400, "Bad Request", "text/plain", body, sizeof(body) - 1);
        return;
    }
    const std::size_t first_line_end = request.find("\r\n");
    const std::string first_line = request.substr(0, first_line_end);
    const std::size_t first_space = first_line.find(' ');
    const std::size_t second_space = first_line.find(' ', first_space + 1);
    if (first_space == std::string::npos || second_space == std::string::npos) {
        return;
    }
    const std::string method = first_line.substr(0, first_space);
    const std::string path = first_line.substr(first_space + 1, second_space - first_space - 1);
    const std::size_t query_at = path.find('?');
    const std::string route = path.substr(0, query_at);
    const std::string body = request.substr(header_end + 4);

    if (method == "GET" && route == "/") {
        send_response(client_fd, 200, "OK", "text/html; charset=utf-8",
                      kIndexHtml, sizeof(kIndexHtml) - 1);
    } else if (method == "GET" && route == "/api/status") {
        const std::string response = status_json();
        send_response(client_fd, 200, "OK", "application/json; charset=utf-8",
                      response.data(), response.size());
    } else if (method == "GET" && route == "/api/frame") {
        std::vector<std::uint8_t> frame;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            frame = snapshot_.y;
        }
        if (frame.empty()) {
            send_response(client_fd, 204, "No Content", "application/octet-stream", nullptr, 0);
        } else {
            send_response(client_fd, 200, "OK", "application/octet-stream",
                          frame.data(), frame.size());
        }
    } else if (method == "GET" && route == "/api/recordings") {
        const std::string response = recordings_json();
        send_response(client_fd, 200, "OK", "application/json; charset=utf-8",
                      response.data(), response.size());
    } else if (method == "GET" && route == "/api/replay") {
        send_replay_frame(client_fd, path);
    } else if (method == "POST" && route == "/api/control") {
        std::string error;
        if (!apply_control(body, error)) {
            const std::string response = json_error(error);
            send_response(client_fd, 400, "Bad Request", "application/json; charset=utf-8",
                          response.data(), response.size());
        } else {
            static const char response[] = "{\"ok\":true}";
            send_response(client_fd, 200, "OK", "application/json; charset=utf-8",
                          response, sizeof(response) - 1);
        }
    } else if (method == "POST" && route == "/api/recording") {
        std::string response;
        std::string error;
        if (!apply_recording(body, response, error)) {
            response = json_error(error);
            send_response(client_fd, 400, "Bad Request", "application/json; charset=utf-8",
                          response.data(), response.size());
        } else {
            send_response(client_fd, 200, "OK", "application/json; charset=utf-8",
                          response.data(), response.size());
        }
    } else {
        static const char response[] = "not found";
        send_response(client_fd, 404, "Not Found", "text/plain", response, sizeof(response) - 1);
    }
}

std::string WebServer::status_json() {
    Snapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot = snapshot_;
    }
    const std::int8_t target = target_control();
    const float target_x = target == 1
                               ? zero_px_
                               : std::clamp(
                                     zero_px_ - static_cast<float>(target) / cm_per_px_,
                                     0.0F,
                                     640.0F);
    std::ostringstream json;
    json << std::fixed << std::setprecision(4)
         << "{\"tx_mode\":\"" << mode_name(tx_mode()) << "\""
         << ",\"target_control\":" << static_cast<int>(target)
         << ",\"manual_cm\":" << manual_cm()
         << ",\"target_x\":" << target_x
         << ",\"detected\":" << (snapshot.detection.valid ? "true" : "false")
         << ",\"cx\":" << snapshot.detection.cx
         << ",\"cy\":" << snapshot.detection.cy
         << ",\"box_width\":" << snapshot.detection.width
         << ",\"box_height\":" << snapshot.detection.height
         << ",\"score\":" << snapshot.detection.score
         << ",\"measured_cm\":" << snapshot.measured_cm
         << ",\"raw_speed_cm_s\":" << snapshot.raw_speed_cm_s
         << ",\"measured_speed_cm_s\":" << snapshot.measured_speed_cm_s
         << ",\"sent_cm\":" << snapshot.sent_cm
         << ",\"sent_speed_cm_s\":" << snapshot.sent_speed_cm_s
         << ",\"transmitting\":" << (snapshot.transmitting ? "true" : "false")
         << ",\"decode_fps\":" << snapshot.decode_fps
         << ",\"infer_fps\":" << snapshot.infer_fps
         << ",\"decoded_frames\":" << snapshot.decoded_frames
         << ",\"inferred_frames\":" << snapshot.inferred_frames
         << ",\"dropped_frames\":" << snapshot.dropped_frames
         << ",\"decoder_resets\":" << snapshot.decoder_resets
         << ",\"tx_frames\":" << snapshot.tx_frames
         << ",\"tx_deadline_misses\":" << snapshot.tx_deadline_misses
         << ",\"recording\":" << (recordings_.recording() ? "true" : "false")
         << ",\"active_recording\":\"" << recordings_.active_id() << "\""
         << ",\"recording_drops\":" << recordings_.dropped_frames()
         << '}';
    return json.str();
}

std::string WebServer::recordings_json() {
    const auto list = recordings_.list();
    std::ostringstream json;
    json << '[';
    for (std::size_t index = 0; index < list.size(); ++index) {
        if (index > 0) {
            json << ',';
        }
        const auto& item = list[index];
        json << "{\"id\":\"" << item.id << "\""
             << ",\"started_unix_ms\":" << item.started_unix_ms
             << ",\"frame_count\":" << item.frame_count
             << ",\"duration_ms\":" << item.duration_ms
             << ",\"width\":" << item.width
             << ",\"height\":" << item.height
             << ",\"fps\":" << item.fps
             << ",\"bytes\":" << item.bytes << '}';
    }
    json << ']';
    return json.str();
}

bool WebServer::apply_control(const std::string& body, std::string& error) {
    const auto values = parse_form(body);
    const auto mode = values.find("mode");
    const auto target = values.find("target_control");
    const auto manual = values.find("manual_cm");
    if (mode == values.end() || target == values.end() || manual == values.end()) {
        error = "缺少控制参数";
        return false;
    }
    std::int8_t target_value = 0;
    float manual_value = 0.0F;
    if (!parse_target(target->second, target_value)) {
        error = "目标必须是 -12 到 +12 的整数";
        return false;
    }
    if (!parse_bounded_float(manual->second, manual_value)) {
        error = "手动位置必须在 -12 到 +12 cm";
        return false;
    }
    TxMode new_mode = TxMode::Stopped;
    if (mode->second == "tracking") {
        new_mode = TxMode::Tracking;
    } else if (mode->second == "manual") {
        new_mode = TxMode::Manual;
    } else if (mode->second != "stopped") {
        error = "未知发送模式";
        return false;
    }
    target_control_.store(target_value, std::memory_order_relaxed);
    manual_cm_.store(manual_value, std::memory_order_relaxed);
    tx_mode_.store(static_cast<int>(new_mode), std::memory_order_release);
    return true;
}

bool WebServer::apply_recording(
    const std::string& body,
    std::string& response,
    std::string& error) {
    const auto values = parse_form(body);
    const auto action = values.find("action");
    if (action == values.end()) {
        error = "缺少记录操作";
        return false;
    }
    if (action->second == "start") {
        std::string id;
        if (!recordings_.start_recording(id, error)) {
            return false;
        }
        response = "{\"ok\":true,\"id\":\"" + id + "\"}";
        return true;
    }
    if (action->second == "stop") {
        recordings_.stop_recording();
        response = "{\"ok\":true}";
        return true;
    }
    error = "未知记录操作";
    return false;
}

bool WebServer::send_replay_frame(int client_fd, const std::string& path) {
    const std::size_t query_at = path.find('?');
    const auto query = parse_form(query_at == std::string::npos ? "" : path.substr(query_at + 1));
    const auto id = query.find("id");
    const auto index = query.find("index");
    if (id == query.end() || index == query.end()) {
        const std::string response = json_error("缺少回放参数");
        send_response(client_fd, 400, "Bad Request", "application/json; charset=utf-8",
                      response.data(), response.size());
        return false;
    }
    std::uint64_t frame_index = 0;
    try {
        std::size_t used = 0;
        frame_index = std::stoull(index->second, &used);
        if (used != index->second.size()) {
            throw std::invalid_argument("trailing data");
        }
    } catch (...) {
        const std::string response = json_error("回放帧编号无效");
        send_response(client_fd, 400, "Bad Request", "application/json; charset=utf-8",
                      response.data(), response.size());
        return false;
    }
    RecordedFrame frame;
    std::string error;
    if (!recordings_.read_frame(id->second, frame_index, frame, error)) {
        const std::string response = json_error(error);
        send_response(client_fd, 404, "Not Found", "application/json; charset=utf-8",
                      response.data(), response.size());
        return false;
    }
    std::ostringstream headers;
    headers << std::fixed << std::setprecision(5)
            << "X-Elapsed-Ms: " << frame.elapsed_ms << "\r\n"
            << "X-Measured-Cm: " << frame.measured_cm << "\r\n"
            << "X-Sent-Cm: " << frame.sent_cm << "\r\n"
            << "X-Target-Control: " << static_cast<int>(frame.target_control) << "\r\n"
            << "X-Detected: " << (frame.detection.valid ? 1 : 0) << "\r\n"
            << "X-Cx: " << frame.detection.cx << "\r\n"
            << "X-Cy: " << frame.detection.cy << "\r\n"
            << "X-Box-Width: " << frame.detection.width << "\r\n"
            << "X-Box-Height: " << frame.detection.height << "\r\n"
            << "X-Score: " << frame.detection.score << "\r\n";
    send_response(client_fd, 200, "OK", "application/octet-stream",
                  frame.y.data(), frame.y.size(), headers.str());
    return true;
}

}  // namespace ti_vision
