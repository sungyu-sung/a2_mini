#!/usr/bin/env python3
"""beep_node_empirestateofmind — TurtleBot4 스피커로 멜로디를 연주하는 노드.

docs/EmpireStateOfMind.pdf 2페이지의 후렴(코러스) 멜로디를 옮긴 것입니다.
조성: ♭6개 = G♭장조 / E♭단조. 로봇은 단음(모노포닉)이라 멜로디 라인만 연주합니다.
현재 SONG에는 악보 판독 + 청음 확인을 마친 "검증 구간"(픽업 "In New" ~ 마디22
"Concrete jungle where dreams are made")만 들어 있습니다. 나머지는 확인 후 이어붙일 예정.

⚠️ 틀린 음은 아래 SONG 리스트에서 음이름만 고치면 됩니다 (샵/플랫 모두 인식).

파라미터:
    topic (str)  : 기본 '/robot2/cmd_audio'
    bpm   (int)  : 기본 90
    loop  (bool) : 기본 True
"""

import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


# ── 음이름 → 주파수(Hz) 테이블 (평균율, A4 = 440Hz) ───────────────────────────
def _build_note_table():
    names = ['C', 'C#', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B']
    table = {'REST': 0}
    for midi in range(36, 96):              # C2 ~ B6
        octave = midi // 12 - 1
        name = names[midi % 12]
        freq = 440.0 * (2 ** ((midi - 69) / 12.0))
        table[f'{name}{octave}'] = int(round(freq))
        # 이명동음(샵 표기) 별칭도 추가: C#=Db, F#=Gb, G#=Ab, A#=Bb, D#=Eb
        sharp_alias = {'C#': 'Db', 'Eb': 'D#', 'Gb': 'F#', 'Ab': 'G#', 'Bb': 'A#'}
        if name in sharp_alias:
            table[f'{sharp_alias[name]}{octave}'] = int(round(freq))
        if name == 'C#':
            table[f'Db{octave}'] = int(round(freq))
    return table


NOTES = _build_note_table()


# ── 곡 데이터: (음이름, 박자) — 박자 1.0 = 4분음표, 0.5 = 8분음표 ─────────────
# "Empire State of Mind" 후렴. 조성: G♭장조 / E♭단조 (♭6개: B E A D G C).
# 악보를 음표 단위로 판독 + 사용자 청음 확인을 거친 "검증된 구간"만 우선 수록.
# (마디 19~20 앞부분은 음정 없는 랩 리듬 슬래시라 제외, 픽업 "In New"부터 시작)
#
#  계이름(사용자 확인): In=레 New=라 York=시 / "Concrete jungle where dreams are[made]"=라 시시시시시 라 높은레
#  → G♭장조 플랫 적용: 레=D♭ 라=A♭ 시=B♭ 높은레=D♭5
SONG = [
    # ── 픽업 "In New" (마디 20 끝) ──
    ('Db4', 0.5),   # In  (8분, 낮은 레)
    ('Ab4', 1.0),   # New (라 — 픽업 8분 + 타이로 다음마디까지 = 4분 길이)
    # ── 마디 21 "York" (시, 길게 유지: 4분+2분 슬러) ──
    ('Bb4', 3.0),   # York
    ('REST', 0.5),
    # ── 마디 22 "Concrete jungle where dreams are made" (전부 8분음표) ──
    ('Ab4', 0.5),   # Con   (라)
    ('Bb4', 0.5),   # crete (시)
    ('Bb4', 0.5),   # jun   (시)
    ('Bb4', 0.5),   # gle   (시)
    ('Bb4', 0.5),   # where (시)
    ('Bb4', 0.5),   # dreams(시)
    ('Ab4', 0.5),   # are   (라)
    ('Db5', 0.5),   # made  (높은레)
]


def _beats_to_duration(beats, bpm):
    sec_total = beats * (60.0 / bpm)
    sec = int(sec_total)
    nanosec = int(round((sec_total - sec) * 1e9))
    return Duration(sec=sec, nanosec=nanosec)


def build_audio_notes(song, bpm):
    """(음이름, 박자) 리스트 → AudioNote 리스트로 변환."""
    notes = []
    for name, beats in song:
        freq = NOTES.get(name, 0)
        notes.append(AudioNote(frequency=int(freq),
                               max_runtime=_beats_to_duration(beats, bpm)))
    return notes


class EmpireStateOfMindNode(Node):
    def __init__(self):
        super().__init__('beep_node_empirestateofmind')

        topic = self.declare_parameter('topic', '/robot2/cmd_audio').value
        self.bpm = self.declare_parameter('bpm', 90).value
        self.loop = self.declare_parameter('loop', True).value

        self.publisher = self.create_publisher(AudioNoteVector, topic, 10)
        self.notes = build_audio_notes(SONG, self.bpm)

        self.song_sec = sum(n.max_runtime.sec + n.max_runtime.nanosec / 1e9
                            for n in self.notes)

        self.get_logger().info(
            f'🎵 Empire State of Mind — topic="{topic}", bpm={self.bpm}, '
            f'loop={self.loop}, notes={len(self.notes)}, '
            f'length≈{self.song_sec:.1f}s')

        # 디스커버리/연결이 자리잡도록 1.5초 후 첫 재생
        self._start_timer = self.create_timer(1.5, self._start_once)
        self._loop_timer = None

    def _start_once(self):
        self._start_timer.cancel()
        self.play()
        if self.loop:
            self._loop_timer = self.create_timer(self.song_sec + 1.0, self.play)

    def play(self):
        msg = AudioNoteVector()
        msg.append = False           # 큐를 비우고 이 곡으로 교체
        msg.notes = self.notes
        self.publisher.publish(msg)
        self.get_logger().info('▶  Empire State of Mind 후렴 재생!')


def main(args=None):
    rclpy.init(args=args)
    node = EmpireStateOfMindNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
