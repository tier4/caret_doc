# treat_drop_as_delay フラグについて

`treat_drop_as_delay=False`とした場合、メッセージがロスト（上書き）されたトレースポイントの箇所で途切れます。
`treat_drop_as_delay=True`とした場合、ロスト箇所を遅延として算出できるように紐付けたフローを出力します。
